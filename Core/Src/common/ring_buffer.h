//
// Created by Michał on 27.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_RING_BUFFER_H
#define DRONE_CONTROLLER_FW_RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** Structure describing the ring buffer. */
typedef struct {
    void* data_buffer;
    size_t cap;
    void* head;
    void* tail;
    size_t len;
    size_t element_size;
} ring_buffer;


/**
 * Initializes the given ring buffer structure.
 *
 * @param ring_buffer pointer to a \ref ring_buffer structure
 * @param data_buffer pointer to a location in memory, where the ring buffer data will be stored
 * @param data_buffer_size size in bytes of the data_buffer
 * @return true if all arguments are valid and the ring buffer is initialized successfully, false otherwise
*/
bool ring_buffer_init(ring_buffer *ring_buffer, void *data_buffer, size_t data_buffer_size, size_t element_size);

/**
 * Clears contents of the given ring buffer.
 *
 * @param ring_buffer pointer to a \ref ring_buffer structure
 * @return true if the ring buffer is cleared successfully, false otherwise
*/
bool ring_buffer_clear(ring_buffer *ring_buffer);

/**
 * Checks if the given ring buffer is empty.
 *
 * @param ring_buffer pointer to a \ref ring_buffer structure
 * @return true if the ring buffer holds no data, false otherwise
*/
bool ring_buffer_is_empty(const ring_buffer *ring_buffer);

/**
 * Gets the length (in bytes) of the data stored in the given ring buffer.
 *
 * @param ring_buffer pointer to a \ref ring_buffer structure
 * @return length (in bytes) of the data stored in the ring buffer
*/
size_t ring_buffer_get_len(const ring_buffer *ring_buffer);

/**
 * Returns the capacity (in bytes) of the given buffer.
 *
 * @param ring_buffer pointer to a \ref ring_buffer structure
 * @return capacity (in bytes) of the ring buffer (how much characters can it store)
*/
size_t ring_buffer_get_capacity(const ring_buffer *ring_buffer);

/**
 * Appends a single character to the ring buffer. The stored data length will be
 * increased by 1.
 *
 * @param ring_buffer pointer to a \ref ring_buffer structure
 * @return true if the character was added successfully, false otherwise
*/
bool ring_buffer_push(ring_buffer *ring_buffer, void *sample);

/**
 * Pulls out a single character from the ring buffer. The stored data length will be
 * decreased by 1.
 *
 * @param ring_buffer pointer to a \ref ring_buffer structure
 * @return true if the character was pulled out successfully, false otherwise
*/
bool ring_buffer_pull(ring_buffer *ring_buffer, void *sample);

#endif //DRONE_CONTROLLER_FW_RING_BUFFER_H
