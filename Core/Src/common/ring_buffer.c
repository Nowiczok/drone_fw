//
// Created by Michał on 27.10.2022.
//

/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include "ring_buffer.h"


bool ring_buffer_init (ring_buffer * ring_buffer, float *data_buffer,
                       size_t data_buffer_size)
{
    assert (ring_buffer);
    assert (data_buffer);
    assert (data_buffer_size > 0);

    if ((ring_buffer) && (data_buffer) && (data_buffer_size > 0))
    {
        ring_buffer->data_buffer = data_buffer;
        ring_buffer->cap = data_buffer_size;
        ring_buffer->head = data_buffer;
        ring_buffer->tail = data_buffer;
        ring_buffer->len = 0;
        return true;
    }

    return false;
}

bool ring_buffer_clear (ring_buffer * ring_buffer)
{
    assert (ring_buffer);

    if (ring_buffer)
    {
        ring_buffer->head = ring_buffer->data_buffer;
        ring_buffer->tail = ring_buffer->data_buffer;
        ring_buffer->len = 0;
        return true;
    }
    return false;
}

bool ring_buffer_is_empty (const ring_buffer * ring_buffer)
{
    assert (ring_buffer);
    if (ring_buffer->len != 0)
        return false;

    return true;
}

size_t ring_buffer_get_len (const ring_buffer * ring_buffer)
{
    assert (ring_buffer);

    if (ring_buffer)
    {
        return ring_buffer->len;
    }
    return 0;

}

size_t ring_buffer_get_capacity (const ring_buffer * ring_buffer)
{
    assert (ring_buffer);

    if (ring_buffer)
    {
        return ring_buffer->cap;
    }
    return 0;
}


bool ring_buffer_push (ring_buffer * ring_buffer, float sample)
{
    assert (ring_buffer);

    if (ring_buffer)
    {
        if (ring_buffer->len < ring_buffer->cap)
        {
            *ring_buffer->head = sample;
            ring_buffer->len++;
            if (ring_buffer->head <
                (ring_buffer->data_buffer +
                 (ring_buffer->cap - 1) * sizeof (float)))
            {
                ring_buffer->head += sizeof (float);
            }
            else
            {
                ring_buffer->head = ring_buffer->data_buffer;
            }
            return true;
        }
    }
    return false;
}

bool ring_buffer_pull (ring_buffer * ring_buffer, float *sample)
{
    assert (ring_buffer);
    assert (sample);

    if ((ring_buffer) && (sample))
    {
        if (!ring_buffer_is_empty(ring_buffer))
        {
            *sample = *ring_buffer->tail;
            ring_buffer->len--;
            if (ring_buffer->tail <
                (ring_buffer->data_buffer +
                 (ring_buffer->cap - 1) * sizeof (float)))
            {
                ring_buffer->tail += sizeof (float);
            }
            else
            {
                ring_buffer->tail = ring_buffer->data_buffer;
            }
            return true;
        }
    }
    return false;
}
