#include "global.h"
#include "board.h"
#include "usb.h"
#include "cli.h"
CBUF(s_usb_write,256);
CBUF(s_usb_read,256);

int lua_main(int argc, char* argv[]);

void main(void)
{
    board_init();

    CBUF_INIT( s_usb_read );
    CBUF_INIT( s_usb_write );

    usb_init( &s_usb_read.cbuf, &s_usb_write.cbuf );
    cli_init( &s_usb_read.cbuf, &s_usb_write.cbuf );

    while( 1 )
    {
        board_sleep();
        cli_event();
    }
}

