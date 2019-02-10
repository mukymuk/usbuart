#include "global.h"
#include "cbuf.h"

void cbuf_init( cbuf_t * p_cbuf, void *pv_buffer, uint32_t size )
{
    p_cbuf->read_ndx = 0;
    p_cbuf->write_ndx = 0;
    p_cbuf->free = size;
    p_cbuf->size = size;
    p_cbuf->buffer = (uint8_t*)pv_buffer;
}

uint32_t cbuf_write( cbuf_t * p_cbuf, const void *pv, uint32_t size )
{
    // copy up to 'size' bytes from 'pv' into the circular buffer
    // return number of bytes actually copied

    uint32_t ndx = 0;
    const uint8_t * p = (const uint8_t *)pv;

    if( p_cbuf->write_lock )
        p_cbuf->lock(p_cbuf);

    while( size && p_cbuf->free )
    {
        p_cbuf->buffer[ p_cbuf->write_ndx++ ] = p[ ndx++ ];
        if( p_cbuf->write_ndx >= p_cbuf->size )
           p_cbuf->write_ndx = 0;
        p_cbuf->free--;
        size--;
    }
    if( p_cbuf->write_lock )
        p_cbuf->unlock(p_cbuf);
    return ndx;
}

uint32_t cbuf_write_aquire( cbuf_t * p_cbuf, void **ppv )
{
    // reserve writable block of contiguous bytes from the circular buffer
    // '*ppv' receives a pointer to the block
    // return number of bytes actually reserved

    if( p_cbuf->write_lock )
        p_cbuf->lock(p_cbuf);

    *ppv = (void*)&p_cbuf->buffer[ p_cbuf->write_ndx ];
    return p_cbuf->size - p_cbuf->write_ndx;

}

void cbuf_write_release( cbuf_t * p_cbuf, uint32_t size )
{
    // release 'size' bytes in circular buffer previously aquired via cbuf_write_aquire()

    p_cbuf->write_ndx += size;
    p_cbuf->free -= size;
    if( p_cbuf->write_ndx >= size )
        p_cbuf->write_ndx = 0;

    if( p_cbuf->write_lock )
        p_cbuf->unlock(p_cbuf);
}

uint32_t cbuf_read( cbuf_t * p_cbuf, void *pv, uint32_t size )
{
    // copy up to 'size' bytes from the circular buffer into 'pv'
    // return number of bytes actually copied

    uint32_t read_count = 0;
    uint32_t ndx = 0;
    uint8_t * p = (uint8_t *)pv;

    if( !p_cbuf->write_lock )
        p_cbuf->lock(p_cbuf);

    while( size && (p_cbuf->free < p_cbuf->size) )
    {
        p[ndx++] = p_cbuf->buffer[ p_cbuf->read_ndx++ ];
        if( p_cbuf->read_ndx >= p_cbuf->size)
            p_cbuf->read_ndx = 0;
        p_cbuf->free++;
        read_count++;
        size--;
    }

    if( !p_cbuf->write_lock )
        p_cbuf->unlock(p_cbuf);

    return read_count;
}

uint32_t cbuf_read_aquire( cbuf_t * p_cbuf, void ** ppv )
{
    // reserve readable block from the circular buffer
    // '*ppv' receives a pointer to the block
    // return number of bytes reserved

    if( !p_cbuf->write_lock )
        p_cbuf->lock(p_cbuf);

    *ppv = (void*)&p_cbuf->buffer[ p_cbuf->read_ndx ];
    return p_cbuf->size - p_cbuf->read_ndx;
}

void cbuf_read_release( cbuf_t * p_cbuf, uint32_t size )
{
    // release 'size' bytes in circular buffer previously aquired via cbuf_read_aquire()
    p_cbuf->read_ndx += size;
    if( p_cbuf->read_ndx >= size )
        p_cbuf->read_ndx = 0;
    p_cbuf->free += size;

    if( !p_cbuf->write_lock )
        p_cbuf->unlock(p_cbuf);

}

void cbuf_read_lock( cbuf_t * p_cbuf, cbuf_sync_t p_lock, cbuf_sync_t p_unlock )
{
    p_cbuf->lock = p_lock;
    p_cbuf->unlock = p_unlock;
    p_cbuf->write_lock = false;
}

void cbuf_write_lock( cbuf_t * p_cbuf, cbuf_sync_t p_lock, cbuf_sync_t p_unlock )
{
    p_cbuf->lock = p_lock;
    p_cbuf->unlock = p_unlock;
    p_cbuf->write_lock = true;
}

