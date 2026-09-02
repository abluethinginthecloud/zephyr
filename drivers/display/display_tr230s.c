/*
 * SPDX-FileCopyrightText: Copyright 2026 Javier Longares Abaiz
 * SPDX-FileCopyrightText: Copyright 2026 A Blue Thing In The Cloud SLU
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file display_tr230s.c
 * @brief Zephyr display driver for the Shanghai TDO TR230S display controller.
 *
 * @author Javier Longares Abaiz
 *
 * @details
 * This driver integrates the TR230S display controller with Zephyr's display
 * subsystem through a MIPI DBI Type C four-wire host interface. It implements
 * rectangular RGB565X transfers, controller-managed backlight brightness,
 * hardware reset, WAIT-line flow control, and display orientation changes.
 *
 * Command, parameter, and pixel transfers use Zephyr's MIPI DBI API. The DBI
 * configuration keeps chip select asserted across the command and data phases
 * of each TR230S transaction and releases the bus when the transaction is
 * complete.
 *
 * The TR230S WAIT input is a controller-ready flow-control signal. It is
 * sampled before every controller transaction and may use either a finite
 * timeout or an unbounded wait, as selected by devicetree.
 *
 * Timing values that depend on the display module are supplied through
 * devicetree properties. The only fixed wait interval in this source file is
 * the WAIT-pin polling cadence, which is a software polling choice rather than
 * a controller timing requirement.
 *
 * The implementation intentionally avoids a full-screen framebuffer. Zephyr
 * supplies partial rendering buffers to tr230s_write(), and the driver streams
 * only the requested rectangle. SRAM consumption is therefore independent of
 * the complete panel resolution.
 *
 * Developed by A Blue Thing In The Cloud SLU.
 * Company website: https://www.abluethinginthecloud.com
 * Author website: https://www.javierlongares.com
 */

#define DT_DRV_COMPAT shtdo_tr230s

#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(display_tr230s, CONFIG_DISPLAY_LOG_LEVEL);

/**
 * @brief TR230S command that programs the controller PWM duty cycle.
 *
 * The command accepts one byte representing brightness as a percentage. This
 * driver limits the value to the inclusive range 0 through 100.
 */
#define TR230S_COMMAND_PWM_DUTY 0x20U

/**
 * @brief TR230S command that programs the inclusive horizontal address range.
 *
 * The payload contains the 16-bit start column followed by the 16-bit end
 * column. Both values are transmitted in big-endian byte order.
 */
#define TR230S_COMMAND_COLUMN_ADDRESS 0x2AU

/**
 * @brief TR230S command that programs the inclusive vertical address range.
 *
 * The payload contains the 16-bit start row followed by the 16-bit end row.
 * Both values are transmitted in big-endian byte order.
 */
#define TR230S_COMMAND_ROW_ADDRESS 0x2BU

/**
 * @brief TR230S command that starts a pixel-data write transaction.
 *
 * The command is followed by RGB565X pixel data within the same held MIPI DBI
 * transaction.
 */
#define TR230S_COMMAND_DISPLAY_DATA 0x2CU

/**
 * @brief TR230S command used to configure mirror and rotation state.
 *
 * The driver maps Zephyr's four display orientations to the controller
 * parameters defined below.
 */
#define TR230S_COMMAND_MIRROR_AND_ROTATION 0xACU

/** Maximum brightness accepted by Zephyr's percentage-based display API. */
#define TR230S_MAXIMUM_BRIGHTNESS_PERCENT 100U

/** PWM percentage transmitted when Zephyr requests display blanking. */
#define TR230S_BLANKED_BRIGHTNESS_PERCENT 0U

/**
 * @brief Interval between consecutive samples of WAIT, in microseconds.
 *
 * WAIT low means that the host must not start another controller transaction.
 * The controller specification does not require this interval between samples;
 * the value controls only the software polling cadence. No delay is introduced
 * when WAIT is already in the ready state.
 */
#define TR230S_READY_POLL_INTERVAL_US 1U

/** Devicetree timeout value selecting an unbounded WAIT-pin poll. */
#define TR230S_READY_TIMEOUT_FOREVER_MS 0U

/** Number of bytes occupied by one RGB565X pixel. */
#define TR230S_PIXEL_SIZE_BYTES 2U

/** Number of 16-bit endpoints in one TR230S address-range payload. */
#define TR230S_ADDRESS_ENDPOINT_COUNT 2U

/** Total number of bytes in a TR230S start/end address-range payload. */
#define TR230S_ADDRESS_RANGE_SIZE_BYTES \
	(TR230S_ADDRESS_ENDPOINT_COUNT * sizeof(uint16_t))

/** Adjustment converting a pixel count to an inclusive final coordinate. */
#define TR230S_INCLUSIVE_RANGE_ADJUSTMENT_PIXELS 1U

/** Logical WAIT GPIO state indicating that a transaction may start. */
#define TR230S_WAIT_READY_STATE 1

/** Number of bits transferred by each MIPI DBI Type C SPI word. */
#define TR230S_DBI_SPI_WORD_SIZE_BITS 8U

/** TR230S parameter selecting the controller's native orientation. */
#define TR230S_ORIENTATION_PARAMETER_NORMAL 0x00U

/** TR230S parameter selecting a 90-degree clockwise rotation. */
#define TR230S_ORIENTATION_PARAMETER_90_DEGREES 0x01U

/** TR230S parameter selecting a 180-degree rotation. */
#define TR230S_ORIENTATION_PARAMETER_180_DEGREES 0x02U

/** TR230S parameter selecting a 270-degree clockwise rotation. */
#define TR230S_ORIENTATION_PARAMETER_270_DEGREES 0x03U

/**
 * @brief Immutable configuration for one TR230S controller instance.
 *
 * One object is generated for each enabled devicetree instance. The object
 * describes the MIPI DBI host, WAIT input, panel dimensions, reset timing, and
 * initial backlight state for that controller.
 */
struct tr230s_configuration {
	/** MIPI DBI host device used for command and display-data transfers. */
	const struct device *mipi_dbi_device;

	/**
	 * Per-display MIPI DBI configuration, including interface mode, clock
	 * rate, word size, chip-select behavior, and SPI target selection.
	 */
	struct mipi_dbi_config dbi_configuration;

	/**
	 * TR230S controller-ready input. The active logical state permits a new
	 * controller transaction. The pin is configured with an input pull-up.
	 */
	struct gpio_dt_spec wait_gpio;

	/** Native horizontal panel resolution before orientation is applied. */
	uint16_t width_pixels;

	/** Native vertical panel resolution before orientation is applied. */
	uint16_t height_pixels;

	/**
	 * Maximum WAIT duration in milliseconds. A value of
	 * TR230S_READY_TIMEOUT_FOREVER_MS selects an unbounded wait.
	 */
	uint32_t ready_timeout_milliseconds;

	/** Delay before hardware reset is asserted, in milliseconds. */
	uint32_t reset_pre_delay_milliseconds;

	/** Duration of the active hardware-reset pulse, in milliseconds. */
	uint32_t reset_pulse_milliseconds;

	/** Delay after hardware reset is released, in milliseconds. */
	uint32_t post_reset_delay_milliseconds;

	/** Initial backlight brightness in the inclusive range 0 through 100. */
	uint8_t default_brightness_percent;
};

/**
 * @brief Mutable runtime state for one TR230S controller instance.
 *
 * The mutex protects multi-command TR230S operations and mutable state from
 * interleaving between application threads. The MIPI DBI host may apply its
 * own lower-level serialization independently.
 */
struct tr230s_runtime_data {
	/** Mutex protecting complete controller operations and mutable state. */
	struct k_mutex access_mutex;

	/** Current Zephyr display orientation accepted by the controller. */
	enum display_orientation orientation;

	/** Requested nonblanked backlight brightness, in percent. */
	uint8_t brightness_percent;

	/** True while Zephyr considers the display blanked. */
	bool display_is_blanked;
};

/**
 * @brief Release the instance mutex without losing the primary result.
 *
 * @param access_mutex Mutex owned by the caller.
 * @param operation_result Result produced by the protected operation.
 *
 * @return @p operation_result when nonzero or when unlock succeeds; otherwise
 * the negative errno value returned by k_mutex_unlock().
 */
static int tr230s_unlock_mutex(struct k_mutex *access_mutex, int operation_result)
{
	int unlock_result;

	unlock_result = k_mutex_unlock(access_mutex);
	if ((operation_result == 0) && (unlock_result != 0)) {
		return unlock_result;
	}

	if ((operation_result != 0) && (unlock_result != 0)) {
		LOG_ERR("Display mutex unlock failed after error %d (%d)",
			operation_result, unlock_result);
	}

	return operation_result;
}

/**
 * @brief Wait until the TR230S WAIT signal permits a new transaction.
 *
 * WAIT is sampled as a logical GPIO value. A ready sample returns immediately.
 * While the controller remains busy, the function polls at
 * TR230S_READY_POLL_INTERVAL_US. A timeout value of zero polls indefinitely.
 *
 * @param dev Initialized TR230S device instance.
 *
 * @retval 0 The controller is ready.
 * @retval -ETIMEDOUT WAIT remained busy until a finite timeout expired.
 * @return Any other negative errno value returned by gpio_pin_get_dt().
 */
static int tr230s_wait_until_ready(const struct device *dev)
{
	const struct tr230s_configuration *configuration;
	int64_t timeout_deadline;
	int wait_state;

	configuration = dev->config;

	if (configuration->ready_timeout_milliseconds ==
	    TR230S_READY_TIMEOUT_FOREVER_MS) {
		for (;;) {
			wait_state = gpio_pin_get_dt(&configuration->wait_gpio);
			if (wait_state < 0) {
				return wait_state;
			}

			if (wait_state == TR230S_WAIT_READY_STATE) {
				return 0;
			}

			k_busy_wait(TR230S_READY_POLL_INTERVAL_US);
		}
	}

	timeout_deadline = k_uptime_get() +
		(int64_t)configuration->ready_timeout_milliseconds;

	do {
		wait_state = gpio_pin_get_dt(&configuration->wait_gpio);
		if (wait_state < 0) {
			return wait_state;
		}

		if (wait_state == TR230S_WAIT_READY_STATE) {
			return 0;
		}

		k_busy_wait(TR230S_READY_POLL_INTERVAL_US);
	} while (k_uptime_get() < timeout_deadline);

	return -ETIMEDOUT;
}

/**
 * @brief Release a held MIPI DBI transaction while preserving its result.
 *
 * The TR230S DBI configuration holds chip select and the underlying SPI lock
 * until mipi_dbi_release() is called. This helper is used after any DBI
 * transaction has started, including error paths.
 *
 * @param dev Initialized TR230S device instance.
 * @param transaction_was_started True after a MIPI DBI operation was called.
 * @param operation_result Result produced before releasing the DBI host.
 *
 * @return @p operation_result when nonzero or when release succeeds; otherwise
 * the negative errno value returned by mipi_dbi_release().
 */
static int tr230s_complete_dbi_transaction(const struct device *dev,
					   bool transaction_was_started,
					   int operation_result)
{
	const struct tr230s_configuration *configuration;
	int release_result;

	if (!transaction_was_started) {
		return operation_result;
	}

	configuration = dev->config;
	release_result = mipi_dbi_release(configuration->mipi_dbi_device,
					  &configuration->dbi_configuration);

	if ((operation_result == 0) && (release_result != 0)) {
		return release_result;
	}

	if ((operation_result != 0) && (release_result != 0)) {
		LOG_ERR("MIPI DBI release failed after error %d (%d)",
			operation_result, release_result);
	}

	return operation_result;
}

/**
 * @brief Send one TR230S command and optional parameter bytes.
 *
 * The caller must hold tr230s_runtime_data::access_mutex. The function waits
 * for WAIT to indicate ready, submits the command and optional parameters as
 * one MIPI DBI command transaction, then releases the held DBI bus.
 *
 * @param dev Initialized TR230S device instance.
 * @param command Controller command byte.
 * @param parameter_buffer Optional command parameter bytes.
 * @param parameter_length Number of bytes in @p parameter_buffer.
 *
 * @retval -EINVAL @p parameter_length is nonzero while @p parameter_buffer is
 * NULL.
 * @return 0 on success, otherwise a negative errno value from WAIT or MIPI DBI.
 */
static int tr230s_write_command_locked(const struct device *dev, uint8_t command,
				       const uint8_t *parameter_buffer,
				       size_t parameter_length)
{
	const struct tr230s_configuration *configuration;
	bool transaction_was_started;
	int operation_result;

	if ((parameter_length != 0U) && (parameter_buffer == NULL)) {
		return -EINVAL;
	}

	configuration = dev->config;
	transaction_was_started = false;

	operation_result = tr230s_wait_until_ready(dev);
	if (operation_result != 0) {
		return operation_result;
	}

	transaction_was_started = true;
	operation_result = mipi_dbi_command_write(
		configuration->mipi_dbi_device,
		&configuration->dbi_configuration,
		command, parameter_buffer, parameter_length);

	return tr230s_complete_dbi_transaction(dev, transaction_was_started,
					       operation_result);
}

/**
 * @brief Encode one inclusive 16-bit TR230S address range in wire order.
 *
 * @param start_position First coordinate included in the address window.
 * @param end_position Last coordinate included in the address window.
 * @param[out] encoded_range Big-endian start and end coordinates.
 */
static void tr230s_encode_address_range(
	uint16_t start_position, uint16_t end_position,
	uint8_t encoded_range[TR230S_ADDRESS_RANGE_SIZE_BYTES])
{
	sys_put_be16(start_position, encoded_range);
	sys_put_be16(end_position, &encoded_range[sizeof(start_position)]);
}

/**
 * @brief Program the address window used by the next pixel transfer.
 *
 * Width and height are pixel counts. TR230S uses inclusive end coordinates,
 * so each final coordinate is start + count - 1. Horizontal and vertical
 * ranges are sent as separate controller transactions.
 *
 * @param dev Initialized TR230S device instance.
 * @param x_position Horizontal coordinate of the first destination pixel.
 * @param y_position Vertical coordinate of the first destination pixel.
 * @param width_pixels Destination width in pixels.
 * @param height_pixels Destination height in pixels.
 *
 * @return 0 on success, otherwise a negative errno value from a command
 * transaction.
 */
static int tr230s_set_memory_area_locked(const struct device *dev,
					 uint16_t x_position,
					 uint16_t y_position,
					 uint16_t width_pixels,
					 uint16_t height_pixels)
{
	uint16_t x_end_position;
	uint16_t y_end_position;
	uint8_t address_parameters[TR230S_ADDRESS_RANGE_SIZE_BYTES];
	int operation_result;

	x_end_position = (uint16_t)((uint32_t)x_position +
		(uint32_t)width_pixels - TR230S_INCLUSIVE_RANGE_ADJUSTMENT_PIXELS);
	y_end_position = (uint16_t)((uint32_t)y_position +
		(uint32_t)height_pixels - TR230S_INCLUSIVE_RANGE_ADJUSTMENT_PIXELS);

	tr230s_encode_address_range(x_position, x_end_position,
				    address_parameters);
	operation_result = tr230s_write_command_locked(
		dev, TR230S_COMMAND_COLUMN_ADDRESS, address_parameters,
		sizeof(address_parameters));
	if (operation_result != 0) {
		return operation_result;
	}

	tr230s_encode_address_range(y_position, y_end_position,
				    address_parameters);

	return tr230s_write_command_locked(
		dev, TR230S_COMMAND_ROW_ADDRESS, address_parameters,
		sizeof(address_parameters));
}

/**
 * @brief Stream RGB565X pixels into the programmed address window.
 *
 * The function waits for WAIT, sends TR230S_COMMAND_DISPLAY_DATA, and writes
 * the associated pixel payload through MIPI DBI while chip select remains
 * asserted. MIPI DBI requires pitch to equal width, so source buffers with
 * row padding are transmitted one visible row at a time.
 *
 * Intermediate row descriptors set frame_incomplete so DBI hosts can preserve
 * frame-level synchronization across the internally split transfer. The final
 * row carries the caller's original frame_incomplete state.
 *
 * @param dev Initialized TR230S device instance.
 * @param descriptor Source rectangle and buffer layout.
 * @param pixel_buffer First RGB565X byte of the source rectangle.
 *
 * @return 0 on success, otherwise a negative errno value from WAIT or MIPI DBI.
 */
static int tr230s_write_pixel_rows_locked(
	const struct device *dev,
	const struct display_buffer_descriptor *descriptor,
	const uint8_t *pixel_buffer)
{
	const struct tr230s_configuration *configuration;
	struct display_buffer_descriptor dbi_descriptor;
	bool transaction_was_started;
	size_t row_stride_bytes;
	size_t row_write_bytes;
	int operation_result;

	configuration = dev->config;
	transaction_was_started = false;
	row_stride_bytes = (size_t)descriptor->pitch * TR230S_PIXEL_SIZE_BYTES;
	row_write_bytes = (size_t)descriptor->width * TR230S_PIXEL_SIZE_BYTES;

	operation_result = tr230s_wait_until_ready(dev);
	if (operation_result != 0) {
		return operation_result;
	}

	transaction_was_started = true;
	operation_result = mipi_dbi_command_write(
		configuration->mipi_dbi_device,
		&configuration->dbi_configuration,
		TR230S_COMMAND_DISPLAY_DATA, NULL, 0U);

	if (operation_result == 0) {
		dbi_descriptor.width = descriptor->width;
		dbi_descriptor.pitch = descriptor->width;

		if (descriptor->pitch == descriptor->width) {
			dbi_descriptor.height = descriptor->height;
			dbi_descriptor.buf_size = (uint32_t)(
				row_write_bytes * (size_t)descriptor->height);
			dbi_descriptor.frame_incomplete =
				descriptor->frame_incomplete;

			operation_result = mipi_dbi_write_display(
				configuration->mipi_dbi_device,
				&configuration->dbi_configuration,
				pixel_buffer, &dbi_descriptor,
				PIXEL_FORMAT_RGB_565X);
		} else {
			dbi_descriptor.height = 1U;
			dbi_descriptor.buf_size = (uint32_t)row_write_bytes;

			for (size_t row_index = 0U;
			     (row_index < (size_t)descriptor->height) &&
			     (operation_result == 0);
			     ++row_index) {
				dbi_descriptor.frame_incomplete =
					((row_index + 1U) <
					 (size_t)descriptor->height) ||
					descriptor->frame_incomplete;

				operation_result = mipi_dbi_write_display(
					configuration->mipi_dbi_device,
					&configuration->dbi_configuration,
					&pixel_buffer[row_index *
						      row_stride_bytes],
					&dbi_descriptor,
					PIXEL_FORMAT_RGB_565X);
			}
		}
	}

	return tr230s_complete_dbi_transaction(dev, transaction_was_started,
					       operation_result);
}

/**
 * @brief Return logical display dimensions for the current orientation.
 *
 * Native width and height are exchanged for 90-degree and 270-degree
 * orientations. Normal and 180-degree orientations preserve them.
 *
 * @param dev Initialized TR230S device instance.
 * @param[out] width_pixels Logical horizontal resolution.
 * @param[out] height_pixels Logical vertical resolution.
 */
static void tr230s_get_logical_resolution(const struct device *dev,
					  uint16_t *width_pixels,
					  uint16_t *height_pixels)
{
	const struct tr230s_configuration *configuration;
	const struct tr230s_runtime_data *runtime_data;

	configuration = dev->config;
	runtime_data = dev->data;

	if ((runtime_data->orientation == DISPLAY_ORIENTATION_ROTATED_90) ||
	    (runtime_data->orientation == DISPLAY_ORIENTATION_ROTATED_270)) {
		*width_pixels = configuration->height_pixels;
		*height_pixels = configuration->width_pixels;
	} else {
		*width_pixels = configuration->width_pixels;
		*height_pixels = configuration->height_pixels;
	}
}

/**
 * @brief Validate a rectangular display-write request before hardware access.
 *
 * The rectangle must be nonempty, pitch must cover each visible row, the
 * destination must fit the current logical resolution, and buf_size must cover
 * every source byte that the driver will access. Buffer calculations use
 * 64-bit arithmetic before comparison with the API and platform limits.
 *
 * @param dev Initialized TR230S device instance.
 * @param x_position Horizontal destination coordinate.
 * @param y_position Vertical destination coordinate.
 * @param descriptor Source rectangle and allocation description.
 * @param pixel_buffer Caller-owned RGB565X source buffer.
 *
 * @retval 0 The request is valid.
 * @retval -EINVAL A pointer, dimension, pitch, destination range, or source
 * buffer size is invalid.
 */
static int tr230s_validate_write_request(
	const struct device *dev, uint16_t x_position, uint16_t y_position,
	const struct display_buffer_descriptor *descriptor,
	const void *pixel_buffer)
{
	uint16_t logical_height_pixels;
	uint16_t logical_width_pixels;
	uint64_t required_buffer_size;
	uint64_t row_stride_bytes;
	uint64_t row_write_bytes;

	if ((descriptor == NULL) || (pixel_buffer == NULL)) {
		return -EINVAL;
	}

	if ((descriptor->width == 0U) || (descriptor->height == 0U) ||
	    (descriptor->pitch < descriptor->width)) {
		return -EINVAL;
	}

	tr230s_get_logical_resolution(dev, &logical_width_pixels,
				      &logical_height_pixels);

	if (((uint32_t)x_position + (uint32_t)descriptor->width >
	     (uint32_t)logical_width_pixels) ||
	    ((uint32_t)y_position + (uint32_t)descriptor->height >
	     (uint32_t)logical_height_pixels)) {
		return -EINVAL;
	}

	row_stride_bytes =
		(uint64_t)descriptor->pitch * TR230S_PIXEL_SIZE_BYTES;
	row_write_bytes =
		(uint64_t)descriptor->width * TR230S_PIXEL_SIZE_BYTES;
	required_buffer_size =
		((uint64_t)descriptor->height - 1U) * row_stride_bytes +
		row_write_bytes;

	if ((required_buffer_size > (uint64_t)descriptor->buf_size) ||
	    (required_buffer_size > (uint64_t)SIZE_MAX)) {
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Write a rectangular RGB565X buffer through Zephyr's display API.
 *
 * Validation occurs before the instance mutex is acquired. Once locked, the
 * column range, row range, display-data command, and pixel payload form one
 * serialized TR230S operation.
 *
 * @param dev Initialized TR230S device instance.
 * @param x_position Horizontal destination coordinate.
 * @param y_position Vertical destination coordinate.
 * @param descriptor Source width, height, pitch, and available bytes.
 * @param pixel_buffer Caller-owned RGB565X source buffer.
 *
 * @retval -EINVAL The request fails validation.
 * @return 0 on success, otherwise a negative errno value from the mutex, GPIO,
 * or MIPI DBI subsystems.
 */
static int tr230s_write(const struct device *dev, uint16_t x_position,
			uint16_t y_position,
			const struct display_buffer_descriptor *descriptor,
			const void *pixel_buffer)
{
	struct tr230s_runtime_data *runtime_data;
	int operation_result;

	operation_result = tr230s_validate_write_request(
		dev, x_position, y_position, descriptor, pixel_buffer);
	if (operation_result != 0) {
		return operation_result;
	}

	runtime_data = dev->data;
	operation_result = k_mutex_lock(&runtime_data->access_mutex, K_FOREVER);
	if (operation_result != 0) {
		return operation_result;
	}

	operation_result = tr230s_set_memory_area_locked(
		dev, x_position, y_position, descriptor->width,
		descriptor->height);
	if (operation_result == 0) {
		operation_result = tr230s_write_pixel_rows_locked(
			dev, descriptor, pixel_buffer);
	}

	return tr230s_unlock_mutex(&runtime_data->access_mutex, operation_result);
}

/**
 * @brief Blank the display by programming zero-percent PWM duty.
 *
 * Blanking affects controller backlight duty only. Pixel memory and the stored
 * nonblanked brightness are preserved.
 *
 * @param dev Initialized TR230S device instance.
 *
 * @return 0 on success, otherwise a negative errno value from mutex, WAIT, or
 * MIPI DBI access.
 */
static int tr230s_blanking_on(const struct device *dev)
{
	struct tr230s_runtime_data *runtime_data;
	const uint8_t blanked_brightness_percent =
		TR230S_BLANKED_BRIGHTNESS_PERCENT;
	int operation_result;

	runtime_data = dev->data;
	operation_result = k_mutex_lock(&runtime_data->access_mutex, K_FOREVER);
	if (operation_result != 0) {
		return operation_result;
	}

	operation_result = tr230s_write_command_locked(
		dev, TR230S_COMMAND_PWM_DUTY, &blanked_brightness_percent,
		sizeof(blanked_brightness_percent));
	if (operation_result == 0) {
		runtime_data->display_is_blanked = true;
	}

	return tr230s_unlock_mutex(&runtime_data->access_mutex, operation_result);
}

/**
 * @brief Unblank the display by restoring the stored brightness.
 *
 * @param dev Initialized TR230S device instance.
 *
 * @return 0 on success, otherwise a negative errno value from mutex, WAIT, or
 * MIPI DBI access.
 */
static int tr230s_blanking_off(const struct device *dev)
{
	struct tr230s_runtime_data *runtime_data;
	int operation_result;

	runtime_data = dev->data;
	operation_result = k_mutex_lock(&runtime_data->access_mutex, K_FOREVER);
	if (operation_result != 0) {
		return operation_result;
	}

	operation_result = tr230s_write_command_locked(
		dev, TR230S_COMMAND_PWM_DUTY,
		&runtime_data->brightness_percent,
		sizeof(runtime_data->brightness_percent));
	if (operation_result == 0) {
		runtime_data->display_is_blanked = false;
	}

	return tr230s_unlock_mutex(&runtime_data->access_mutex, operation_result);
}

/**
 * @brief Set the nonblanked TR230S backlight brightness percentage.
 *
 * A value set while blanked is stored without changing the controller PWM,
 * preserving the blanked state. Otherwise the new duty is applied immediately
 * and retained only after a successful transaction.
 *
 * @param dev Initialized TR230S device instance.
 * @param brightness_percent Requested brightness from 0 through 100 percent.
 *
 * @retval -EINVAL @p brightness_percent exceeds 100.
 * @return 0 on success, otherwise a negative errno value from mutex, WAIT, or
 * MIPI DBI access.
 */
static int tr230s_set_brightness(const struct device *dev,
				 uint8_t brightness_percent)
{
	struct tr230s_runtime_data *runtime_data;
	int operation_result;

	if (brightness_percent > TR230S_MAXIMUM_BRIGHTNESS_PERCENT) {
		return -EINVAL;
	}

	runtime_data = dev->data;
	operation_result = k_mutex_lock(&runtime_data->access_mutex, K_FOREVER);
	if (operation_result != 0) {
		return operation_result;
	}

	if (runtime_data->display_is_blanked) {
		runtime_data->brightness_percent = brightness_percent;
		operation_result = 0;
	} else {
		operation_result = tr230s_write_command_locked(
			dev, TR230S_COMMAND_PWM_DUTY, &brightness_percent,
			sizeof(brightness_percent));
		if (operation_result == 0) {
			runtime_data->brightness_percent = brightness_percent;
		}
	}

	return tr230s_unlock_mutex(&runtime_data->access_mutex, operation_result);
}

/**
 * @brief Validate a pixel format requested through Zephyr's display API.
 *
 * @param dev TR230S device instance. No state is modified.
 * @param pixel_format Requested Zephyr pixel format.
 *
 * @retval 0 @p pixel_format is PIXEL_FORMAT_RGB_565X.
 * @retval -ENOTSUP The requested format is unsupported.
 */
static int tr230s_set_pixel_format(const struct device *dev,
				   enum display_pixel_format pixel_format)
{
	ARG_UNUSED(dev);

	if (pixel_format != PIXEL_FORMAT_RGB_565X) {
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief Set TR230S orientation through Zephyr's display API.
 *
 * The Zephyr orientation is mapped to the corresponding one-byte TR230S
 * mirror/rotation parameter. Runtime state changes only after the command
 * transaction succeeds.
 *
 * @param dev Initialized TR230S device instance.
 * @param orientation Requested Zephyr display orientation.
 *
 * @retval -EINVAL @p orientation is not one of the four supported values.
 * @return 0 on success, otherwise a negative errno value from mutex, WAIT, or
 * MIPI DBI access.
 */
static int tr230s_set_orientation(const struct device *dev,
				  enum display_orientation orientation)
{
	struct tr230s_runtime_data *runtime_data;
	uint8_t orientation_parameter;
	int operation_result;

	switch (orientation) {
	case DISPLAY_ORIENTATION_NORMAL:
		orientation_parameter = TR230S_ORIENTATION_PARAMETER_NORMAL;
		break;
	case DISPLAY_ORIENTATION_ROTATED_90:
		orientation_parameter = TR230S_ORIENTATION_PARAMETER_90_DEGREES;
		break;
	case DISPLAY_ORIENTATION_ROTATED_180:
		orientation_parameter = TR230S_ORIENTATION_PARAMETER_180_DEGREES;
		break;
	case DISPLAY_ORIENTATION_ROTATED_270:
		orientation_parameter = TR230S_ORIENTATION_PARAMETER_270_DEGREES;
		break;
	default:
		return -EINVAL;
	}

	runtime_data = dev->data;
	operation_result = k_mutex_lock(&runtime_data->access_mutex, K_FOREVER);
	if (operation_result != 0) {
		return operation_result;
	}

	if (runtime_data->orientation == orientation) {
		operation_result = 0;
	} else {
		operation_result = tr230s_write_command_locked(
			dev, TR230S_COMMAND_MIRROR_AND_ROTATION,
			&orientation_parameter, sizeof(orientation_parameter));
		if (operation_result == 0) {
			runtime_data->orientation = orientation;
		}
	}

	return tr230s_unlock_mutex(&runtime_data->access_mutex, operation_result);
}

/**
 * @brief Report capabilities and current state of a TR230S instance.
 *
 * The logical resolution reflects the current orientation. The driver
 * advertises RGB565X and the most recently accepted orientation.
 *
 * @param dev Initialized TR230S device instance.
 * @param[out] capabilities Destination for Zephyr display capabilities. A NULL
 * pointer is ignored because this callback cannot report an argument error.
 */
static void tr230s_get_capabilities(const struct device *dev,
				    struct display_capabilities *capabilities)
{
	const struct tr230s_runtime_data *runtime_data;
	uint16_t logical_height_pixels;
	uint16_t logical_width_pixels;

	if (capabilities == NULL) {
		return;
	}

	runtime_data = dev->data;
	tr230s_get_logical_resolution(dev, &logical_width_pixels,
				      &logical_height_pixels);

	(void)memset(capabilities, 0, sizeof(*capabilities));
	capabilities->x_resolution = logical_width_pixels;
	capabilities->y_resolution = logical_height_pixels;
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_565X;
	capabilities->current_pixel_format = PIXEL_FORMAT_RGB_565X;
	capabilities->current_orientation = runtime_data->orientation;
}

/**
 * @brief Initialize one TR230S controller instance.
 *
 * Initialization verifies the MIPI DBI host and WAIT GPIO, validates static
 * panel parameters, initializes runtime state, configures WAIT, and performs
 * the hardware reset through the MIPI DBI controller using the configured
 * pre-reset, pulse, and post-reset timing values.
 *
 * No controller command is transmitted during initialization. After reset and
 * the configured post-reset interval, normal display API operations provide
 * the first controller transactions.
 *
 * @param dev TR230S device instance being initialized.
 *
 * @retval 0 Initialization completed successfully.
 * @retval -ENODEV The MIPI DBI host or WAIT GPIO controller is not ready.
 * @retval -ENOTSUP The configured MIPI DBI mode is unsupported by this driver.
 * @retval -EINVAL Static panel or brightness parameters are invalid.
 * @return Any other negative errno value returned by GPIO or MIPI DBI reset.
 */
static int tr230s_initialize(const struct device *dev)
{
	const struct tr230s_configuration *configuration;
	struct tr230s_runtime_data *runtime_data;
	int operation_result;

	configuration = dev->config;
	runtime_data = dev->data;

	if (!device_is_ready(configuration->mipi_dbi_device) ||
	    !gpio_is_ready_dt(&configuration->wait_gpio)) {
		return -ENODEV;
	}

	if (configuration->dbi_configuration.mode !=
	    MIPI_DBI_MODE_SPI_4WIRE) {
		return -ENOTSUP;
	}

	if ((configuration->width_pixels == 0U) ||
	    (configuration->height_pixels == 0U) ||
	    (configuration->reset_pulse_milliseconds == 0U) ||
	    (configuration->default_brightness_percent >
	     TR230S_MAXIMUM_BRIGHTNESS_PERCENT)) {
		return -EINVAL;
	}

	k_mutex_init(&runtime_data->access_mutex);
	runtime_data->orientation = DISPLAY_ORIENTATION_NORMAL;
	runtime_data->brightness_percent =
		configuration->default_brightness_percent;
	runtime_data->display_is_blanked = false;

	operation_result = gpio_pin_configure_dt(
		&configuration->wait_gpio, GPIO_INPUT | GPIO_PULL_UP);
	if (operation_result != 0) {
		return operation_result;
	}

	if (configuration->reset_pre_delay_milliseconds != 0U) {
		k_sleep(K_MSEC(configuration->reset_pre_delay_milliseconds));
	}

	operation_result = mipi_dbi_reset(
		configuration->mipi_dbi_device,
		configuration->reset_pulse_milliseconds);
	if (operation_result != 0) {
		return operation_result;
	}

	if (configuration->post_reset_delay_milliseconds != 0U) {
		k_sleep(K_MSEC(configuration->post_reset_delay_milliseconds));
	}

	LOG_INF("TR230S initialized: %ux%u, RGB565X, MIPI DBI Type C, %u Hz",
		(unsigned int)configuration->width_pixels,
		(unsigned int)configuration->height_pixels,
		(unsigned int)configuration->dbi_configuration.config.frequency);

	return 0;
}

/**
 * @brief Zephyr display API implementation shared by all TR230S instances.
 */
static DEVICE_API(display, tr230s_display_api) = {
	.blanking_on = tr230s_blanking_on,
	.blanking_off = tr230s_blanking_off,
	.write = tr230s_write,
	.set_brightness = tr230s_set_brightness,
	.get_capabilities = tr230s_get_capabilities,
	.set_pixel_format = tr230s_set_pixel_format,
	.set_orientation = tr230s_set_orientation,
};

/**
 * @brief Instantiate one enabled TR230S devicetree instance.
 *
 * Each instance receives compile-time range checks, one immutable controller
 * configuration, one mutable runtime object, and a Zephyr display device.
 * The MIPI DBI configuration uses four-wire Type C transfers with 8-bit words,
 * MSB-first order, and held/locked chip select until explicit release.
 *
 * @param instance Zero-based devicetree instance number.
 */
#define TR230S_DEFINE(instance)                                                   \
	BUILD_ASSERT(DT_INST_PROP(instance, width) > 0,                           \
		     "TR230S width must be positive");                            \
	BUILD_ASSERT(DT_INST_PROP(instance, width) <= UINT16_MAX,                  \
		     "TR230S width exceeds the display API range");               \
	BUILD_ASSERT(DT_INST_PROP(instance, height) > 0,                          \
		     "TR230S height must be positive");                           \
	BUILD_ASSERT(DT_INST_PROP(instance, height) <= UINT16_MAX,                 \
		     "TR230S height exceeds the display API range");              \
	BUILD_ASSERT(DT_INST_PROP(instance, reset_pulse_ms) > 0,                  \
		     "TR230S reset pulse must be positive");                      \
	BUILD_ASSERT(DT_INST_PROP(instance, default_brightness) <=                 \
			     TR230S_MAXIMUM_BRIGHTNESS_PERCENT,                       \
		     "TR230S default brightness exceeds 100 percent");            \
	                                                                            \
	static const struct tr230s_configuration tr230s_configuration_##instance = { \
		.mipi_dbi_device = DEVICE_DT_GET(DT_INST_PARENT(instance)),          \
		.dbi_configuration = MIPI_DBI_CONFIG_DT_INST(                       \
			instance, SPI_OP_MODE_CONTROLLER | SPI_TRANSFER_MSB |         \
				SPI_WORD_SET(TR230S_DBI_SPI_WORD_SIZE_BITS) |          \
				SPI_HOLD_ON_CS | SPI_LOCK_ON, 0),                      \
		.wait_gpio = GPIO_DT_SPEC_INST_GET(instance, wait_gpios),            \
		.width_pixels = (uint16_t)DT_INST_PROP(instance, width),              \
		.height_pixels = (uint16_t)DT_INST_PROP(instance, height),            \
		.ready_timeout_milliseconds =                                         \
			(uint32_t)DT_INST_PROP(instance, ready_timeout_ms),             \
		.reset_pre_delay_milliseconds =                                       \
			(uint32_t)DT_INST_PROP(instance, reset_pre_delay_ms),           \
		.reset_pulse_milliseconds =                                           \
			(uint32_t)DT_INST_PROP(instance, reset_pulse_ms),               \
		.post_reset_delay_milliseconds =                                      \
			(uint32_t)DT_INST_PROP(instance, post_reset_delay_ms),          \
		.default_brightness_percent =                                         \
			(uint8_t)DT_INST_PROP(instance, default_brightness),            \
	};                                                                          \
	                                                                            \
	static struct tr230s_runtime_data tr230s_runtime_data_##instance;           \
	                                                                            \
	DEVICE_DT_INST_DEFINE(instance, tr230s_initialize, NULL,                    \
			      &tr230s_runtime_data_##instance,                        \
			      &tr230s_configuration_##instance, POST_KERNEL,          \
			      CONFIG_DISPLAY_INIT_PRIORITY, &tr230s_display_api);

DT_INST_FOREACH_STATUS_OKAY(TR230S_DEFINE)
