#ifndef __CBUF_H__
#define __CBUF_H__

struct _cbuf_t;

typedef void (*cbuf_sync_t)( struct _cbuf_t *p_cbuf );

typedef struct _cbuf_t
{
    int32_t                 free;
    int32_t                 write_ndx;
    int32_t                 read_ndx;
    uint8_t *               buffer;
    int32_t                 size : 31;
    uint32_t                write_lock;
    cbuf_sync_t             lock;
    cbuf_sync_t             unlock;
}
cbuf_t;

void cbuf_init( cbuf_t * p_cirbuf, void *pv_buffer, uint32_t size );
uint32_t cbuf_write( cbuf_t * p_cbuf, const void *pv, uint32_t size );
uint32_t cbuf_write_aquire( cbuf_t * p_cbuf, void **ppv );
void cbuf_write_release( cbuf_t * p_cbuf, uint32_t size );
uint32_t cbuf_read( cbuf_t * p_cbuf, void *pv, uint32_t size );
uint32_t cbuf_read_aquire( cbuf_t * p_cbuf, void ** ppv );
void cbuf_read_release( cbuf_t * p_cbuf, uint32_t size );
void cbuf_read_lock( cbuf_t * p_cbuf, cbuf_sync_t p_lock, cbuf_sync_t p_unlock );
void cbuf_write_lock( cbuf_t * p_cbuf, cbuf_sync_t p_lock, cbuf_sync_t p_unlock );

#define CBUF(name,size) typedef struct\
{\
    cbuf_t cbuf;\
    uint8_t buffer[size];\
} name##_t;\
static name##_t name
#define CBUF_INIT(name) cbuf_init( &name.cbuf, name.buffer, sizeof(name.buffer) )

#endif
