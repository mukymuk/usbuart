#include "global.h"
#include "cli.h"

#define MAX_LINE_SIZE   64

static cbuf_t *s_p_cbuf_write;
static cbuf_t *s_p_cbuf_read;

void cli_init( cbuf_t * p_cbuf_read, cbuf_t * p_cbuf_write  )
{
    s_p_cbuf_write = p_cbuf_write;
    s_p_cbuf_read = p_cbuf_read;
}

#define LF  '\n'
#define ESC 0x1B

void cli_event(void)
{
    uint8_t c;
    static uint8_t cmd[MAX_LINE_SIZE];
    static uint8_t ndx;

    while( cbuf_read( s_p_cbuf_read, &c, sizeof(c) ) )
    {
        cbuf_write( s_p_cbuf_write, &c, sizeof(c) );
    }
}
