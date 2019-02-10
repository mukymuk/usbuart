#include "global.h"
#include "usb.h"

#include "mxc_device.h"
#include "pwrman_regs.h"
#include "clkman_regs.h"
#include "usb_regs.h"

#undef DBG
#define DBG(x)

#define ACM_SET_LINE_CODING         0x20
#define ACM_GET_LINE_CODING         0x21

#define BMATTRIBUTES_TRANSFER_TYPE_BULK         2
#define BMATTRIBUTES_TRANSFER_TYPE_INTERRUPT    3

#define BMATTRIBUTES_SYNC_TYPE_NONE             (0<<2)
#define BMATTRIBUTES_SYNC_TYPE_ASYNC            (1<<2)
#define BMATTRIBUTES_SYNC_TYPE_ADAPTIVE         (2<<2)
#define BMATTRIBUTES_SYNC_TYPE_SYNC             (3<<2)

#define BMATTRIBUTES_USAGE_TYPE_DATA            (0<<4)
#define BMATTRIBUTES_USAGE_TYPE_FEEDBACK        (1<<4)
#define BMATTRIBUTES_USAGE_TYPE_IMPLICIT_FEEDBACK (2<<4)

#define DESCRIPTOR_TYPE_DEVICE          1
#define DESCRIPTOR_TYPE_CONFIGURATION   2
#define DESCRIPTOR_TYPE_STRING          3

#define SETUP_REQUEST_TYPE_DIRECTION_HOST_TO_DEVICE 0
#define SETUP_REQUEST_TYPE_DIRECTION_DEVICE_TO_HOST 1

#define SETUP_REQUEST_TYPE_STANDARD   0
#define SETUP_REQUEST_TYPE_CLASS      1
#define SETUP_REQUEST_TYPE_VENDOR     2

#define SETUP_REQUEST_TYPE_RECIPIENT_DEVICE       0
#define SETUP_REQUEST_TYPE_RECIPIENT_INTERFACE    1
#define SETUP_REQUEST_TYPE_RECIPIENT_ENDPOINT     2

#define SETUP_REQUEST_GET_STATUS            0
#define SETUP_REQUEST_CLEAR_FEATURE         1
#define SETUP_REQUEST_SET_FEATURE           3
#define SETUP_REQUEST_SET_ADDRESS           5
#define SETUP_REQUEST_GET_DESCRIPTOR        6
#define SETUP_REQUEST_SET_DESCRIPTOR        7
#define SETUP_REQUEST_GET_CONFIGURATION     8
#define SETUP_REQUEST_SET_CONFIGURATION     9
#define SETUP_REQUEST_GET_INTERFACE         10
#define SETUP_REQUEST_SET_INTERFACE         0x11

#define DESC_DEVICE               1
#define DESC_CONFIG               2
#define DESC_INTERFACE            4
#define DESC_ENDPOINT             5

#define ENDPOINT_CONTROL          0
#define ENDPOINT_ACM_BULK_OUT     1
#define ENDPOINT_ACM_BULK_IN      2
#define ENDPOINT_BULK_COUNT       2
#define ENDPOINT_CDC_INT_IN       3

#pragma pack(1)

typedef struct
{
    uint8_t     recipient : 5;
    uint8_t     type      : 2;
    uint8_t     direction : 1;
}
usb_setup_request_type_t;

typedef struct
{
    usb_setup_request_type_t        bmRequestType;
    uint8_t                         bRequest;
    uint16_t                        wValue;
    uint16_t                        wIndex;
    uint16_t                        wLength;
}
usb_setup_t;

typedef union
{
    usb_setup_t setup;
    uint32_t    reg[2];
}
usb_setup_reg_t;

typedef struct
{
    uint32_t  dwDTERate;
    uint8_t   bCharFormat;
    uint8_t   bParityType;
    uint8_t   bDataBits;
}
acm_line_coding_t;

typedef struct
{
    volatile uint32_t size;
    volatile uint32_t address;
}
usb_buffer_t;

typedef struct
{
    usb_buffer_t      buffer[2];
}
usb_endpoint_buffer_t;

typedef struct
{
    usb_endpoint_buffer_t      out;
    usb_endpoint_buffer_t      in;
}
usb_endpoint0_buffer_t;

typedef struct
{
    usb_endpoint0_buffer_t    endpoint0;
    usb_endpoint_buffer_t     endpoint[ENDPOINT_BULK_COUNT];
}
usb_sie_t;

typedef struct
{
    uint8_t      bLength;
    uint8_t      bDescriptorType;
}
usb_descriptor_header_t;

typedef struct
{
    usb_descriptor_header_t header;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
}
usb_device_descriptor_t;

typedef struct
{
    usb_descriptor_header_t header;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
}
usb_configuration_descriptor_header_t;

typedef struct
{
    usb_descriptor_header_t header;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
}
usb_interface_descriptor_t;

typedef struct
{
    usb_descriptor_header_t header;
    uint8_t                 bEndpointAddress;
    uint8_t                 bmAttributes;
    uint16_t                wMaxPacketSize;
    uint8_t                 bInterval;
}
usb_endpoint_descriptor_t;

typedef struct
{
    uint8_t bFunctionalLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubtype;
}
usb_class_descriptor_header_t;

typedef struct
{
    usb_class_descriptor_header_t   header;
    uint8_t                         bcdCDC[2];
}
usb_cdc_header_functional_t;

typedef struct
{
    usb_class_descriptor_header_t   header;
    uint8_t                         bmCapabilities;
    uint8_t                         bmDataInterface;
}
usb_cdc_call_management_t;

typedef struct
{
    usb_class_descriptor_header_t   header;
    uint8_t                         bmCapabilities;
}
usb_cdc_abstract_control_t;

typedef struct
{
    usb_class_descriptor_header_t   header;
    uint8_t                         bmMasterInterface;
    uint8_t                         bmSlaveInterface0;
}
usb_cdc_union_functional_t;

typedef struct
{
    usb_configuration_descriptor_header_t   header;
    usb_interface_descriptor_t              cdc_if;
    usb_cdc_header_functional_t             header_functional;
    usb_cdc_call_management_t               call_management;
    usb_cdc_abstract_control_t              abstract_control;
    usb_cdc_union_functional_t              union_functional;
    usb_endpoint_descriptor_t               ep_int_in;
    usb_interface_descriptor_t              data_if;
    usb_endpoint_descriptor_t               ep_bulk_out;
    usb_endpoint_descriptor_t               ep_bulk_in;
}
usb_configuration_descriptor_t;

typedef struct
{
    usb_device_descriptor_t         device;
    usb_configuration_descriptor_t  configuration;
}
descriptors_t;

#pragma pack()

typedef struct
{
    uint8_t                 buffer[MXC_USB_MAX_PACKET];
    usb_buffer_t * const    sie;
    uint8_t *               p_data;
    uint16_t                size;
}
endpoint_t;

static const uint8_t s_lang_id_desc[4] =
{
    4, 3, 9, 4
};

static const uint8_t s_mfg_id_desc[8] =
{
    8, 3,
    'b', 0,
    'o', 0,
    'e', 0,
};

static const uint8_t s_prod_id_desc[4] =
{
    2, 3,
    '1', 0,
};

#define CDC_HEADER(length,subtype) { .bFunctionalLength = length, .bDescriptorType = 0x24, .bDescriptorSubtype = subtype }
#define DESC_HEADER(length,type)    { .bLength = length, .bDescriptorType = type }

static const descriptors_t s_descriptors =
{
    .device =
    {
        .header = DESC_HEADER( sizeof(s_descriptors.device), DESC_DEVICE ),
        .bcdUSB = 0x0110,
        .bDeviceClass = 0x02,
        .bDeviceSubClass = 0,
        .bDeviceProtocol = 0,
        .bMaxPacketSize = MXC_USB_MAX_PACKET,
        .idVendor = 0x0B6A,
        .idProduct = 0x003C,
        .bcdDevice = 0x100,
        .iManufacturer = 1,
        .iProduct = 2,
        .iSerialNumber = 0,
        .bNumConfigurations = 1
    },
    .configuration =
    {
        .header =
        {
            .header = DESC_HEADER( sizeof(s_descriptors.configuration.header), DESC_CONFIG ),
            .wTotalLength = sizeof(s_descriptors.configuration),
            .bNumInterfaces = 2,
            .bConfigurationValue = 1,
            .iConfiguration = 0,
            .bmAttributes = 0x80, // bus powered
            .bMaxPower = 50 // 100mA
        },
        .cdc_if =
        {
            .header = DESC_HEADER( sizeof(s_descriptors.configuration.cdc_if), DESC_INTERFACE ),
            .bInterfaceNumber = 0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 1,
            .bInterfaceClass = 2,
            .bInterfaceSubClass = 2,
            .bInterfaceProtocol = 1,
            .iInterface = 0
        },
        .header_functional =
        {
            .header = CDC_HEADER(sizeof(s_descriptors.configuration.header_functional),0),
            .bcdCDC = { 0x10, 0x01 }
        },
        .call_management =
        {
            .header = CDC_HEADER(sizeof(s_descriptors.configuration.call_management),1),
            .bmCapabilities = 0,
            .bmDataInterface = 1
        },
        .abstract_control =
        {
            .header = CDC_HEADER(sizeof(s_descriptors.configuration.abstract_control),2),
            .bmCapabilities = 0
        },
        .union_functional =
        {
            .header = CDC_HEADER(sizeof(s_descriptors.configuration.union_functional),6),
            .bmMasterInterface = 0,
            .bmSlaveInterface0 = 1
        },
        .ep_int_in =
        {
            .header = DESC_HEADER(sizeof(s_descriptors.configuration.ep_int_in),DESC_ENDPOINT),
            .bEndpointAddress = 0x80 | ENDPOINT_CDC_INT_IN,
            .bmAttributes = BMATTRIBUTES_TRANSFER_TYPE_INTERRUPT|BMATTRIBUTES_SYNC_TYPE_NONE|BMATTRIBUTES_USAGE_TYPE_DATA,
            .wMaxPacketSize = MXC_USB_MAX_PACKET,
            .bInterval = 0xFF
        },
        .data_if =
        {
            .header = DESC_HEADER( sizeof(s_descriptors.configuration.data_if), DESC_INTERFACE ),
            .bInterfaceNumber = 1,
            .bAlternateSetting = 0,
            .bNumEndpoints = 2,
            .bInterfaceClass = 10,
            .bInterfaceSubClass = 0,
            .bInterfaceProtocol = 0,
            .iInterface = 0
        },
        .ep_bulk_out =
        {
            .header = DESC_HEADER(sizeof(s_descriptors.configuration.ep_bulk_out),DESC_ENDPOINT),
            .bEndpointAddress = ENDPOINT_ACM_BULK_OUT,
            .bmAttributes = BMATTRIBUTES_TRANSFER_TYPE_BULK|BMATTRIBUTES_SYNC_TYPE_NONE|BMATTRIBUTES_USAGE_TYPE_DATA,
            .wMaxPacketSize = MXC_USB_MAX_PACKET,
            .bInterval = 0
        },
        .ep_bulk_in =
        {
            .header = DESC_HEADER(sizeof(s_descriptors.configuration.ep_bulk_in),DESC_ENDPOINT),
            .bEndpointAddress = 0x80 | ENDPOINT_ACM_BULK_IN,
            .bmAttributes = BMATTRIBUTES_TRANSFER_TYPE_BULK|BMATTRIBUTES_SYNC_TYPE_NONE|BMATTRIBUTES_USAGE_TYPE_DATA,
            .wMaxPacketSize = MXC_USB_MAX_PACKET,
            .bInterval = 0
        },
    }
};

static __attribute__ ((aligned (512))) usb_sie_t    s_usb_sie;

static endpoint_t s_ep_out;
static endpoint_t s_ep_in;
static endpoint_t s_ep_control;

static cbuf_t *s_cbuf_read;
static cbuf_t *s_cbuf_write;
static uint32_t s_read_overflow;

usb_buffer_t * const sie_bulk_out = &s_usb_sie.endpoint[ENDPOINT_ACM_BULK_OUT-1].buffer[0];
usb_buffer_t * const sie_bulk_in = &s_usb_sie.endpoint[ENDPOINT_ACM_BULK_IN-1].buffer[0];
usb_buffer_t * const sie_ctrl_out = &s_usb_sie.endpoint0.out.buffer[0];
usb_buffer_t * const sie_ctrl_in = &s_usb_sie.endpoint0.in.buffer[0];

static void inline usb_stall( uint8_t ep )
{
    MXC_USB->ep[ep] |= MXC_F_USB_EP_STALL | MXC_F_USB_EP_ST_STALL;
}

static void inline usb_ack( uint8_t ep )
{
    MXC_USB->ep[ep] |= MXC_F_USB_EP_ST_ACK;
}

static void lock( cbuf_t *p_cbuf )
{
    if( p_cbuf == s_cbuf_write )
        MXC_USB->ep[ENDPOINT_ACM_BULK_IN] &= ~MXC_F_USB_EP_INT_EN;
    else
        MXC_USB->ep[ENDPOINT_ACM_BULK_OUT] &= ~MXC_F_USB_EP_INT_EN;
}

static bool s_ining;

static void write_bulk_in( void )
{
    uint32_t size = cbuf_read( s_cbuf_write, s_ep_in.buffer, MXC_USB_MAX_PACKET );
    sie_bulk_in->size = size;
    if( size )
    {
        s_ining = true;
        sie_bulk_in->size = size;
        MXC_USB->in_owner = (1 << ENDPOINT_ACM_BULK_IN);    // return buffer to usb sie control
    }
    else
        s_ining = false;
    usb_ack(ENDPOINT_ACM_BULK_IN);
}

static void unlock( cbuf_t *p_cbuf )
{
    if( p_cbuf == s_cbuf_write )
    {
        if( !s_ining )
            write_bulk_in();
        MXC_USB->ep[ENDPOINT_ACM_BULK_IN] |= MXC_F_USB_EP_INT_EN;
    }
    else
        MXC_USB->ep[ENDPOINT_ACM_BULK_OUT] |= MXC_F_USB_EP_INT_EN;
}

static void inline ready_bulk_out(void)
{
    sie_bulk_out->size = MXC_USB_MAX_PACKET;  // reset for subsequent reads
    MXC_USB->out_owner = (1<<ENDPOINT_ACM_BULK_OUT);    // return buffer to usb sie control
}

void usb_init( cbuf_t * p_cbuf_read, cbuf_t * p_cbuf_write )
{
    uint8_t i;

    cbuf_write_lock( p_cbuf_write, lock, unlock );
    cbuf_read_lock( p_cbuf_read, lock, unlock );

    s_cbuf_read = p_cbuf_read;
    s_cbuf_write = p_cbuf_write;

    MXC_CLKMAN->clk_ctrl = MXC_F_CLKMAN_CLK_CTRL_USB_CLOCK_ENABLE;
    MXC_PWRMAN->pwr_rst_ctrl |= MXC_F_PWRMAN_PWR_RST_CTRL_USB_POWERED;

    sie_bulk_out->address = (int32_t)s_ep_out.buffer;
    sie_bulk_in->address = (int32_t)s_ep_in.buffer;
    sie_ctrl_in->address = (int32_t)s_ep_control.buffer;
    sie_ctrl_out->address = (int32_t)s_ep_control.buffer;

    MXC_USB->ep[ENDPOINT_CONTROL] = MXC_F_USB_EP_INT_EN | MXC_V_USB_EP_DIR_CONTROL << MXC_F_USB_EP_DIR_POS;
    MXC_USB->ep[ENDPOINT_ACM_BULK_IN] = MXC_F_USB_EP_INT_EN | MXC_V_USB_EP_DIR_IN << MXC_F_USB_EP_DIR_POS;
    MXC_USB->ep[ENDPOINT_ACM_BULK_OUT] = MXC_F_USB_EP_INT_EN | MXC_V_USB_EP_DIR_OUT << MXC_F_USB_EP_DIR_POS;

    MXC_USB->cn = 1;
    MXC_USB->ep_base = (uint32_t)&s_usb_sie;
    MXC_USB->dev_inten = MXC_F_USB_DEV_INTEN_SETUP | MXC_F_USB_DEV_INTEN_EP_IN | MXC_F_USB_DEV_INTEN_EP_OUT;
    MXC_USB->dev_cn = (MXC_F_USB_DEV_CN_CONNECT | MXC_F_USB_DEV_CN_FIFO_MODE);

    ready_bulk_out();
    NVIC_EnableIRQ(USB_IRQn);
}

static void write_control_in( void )
{
    uint32_t size = MIN(s_ep_control.size,MXC_USB_MAX_PACKET);
    if( size )
    {
        sie_ctrl_in->size = size;
        memcpy( s_ep_control.buffer, s_ep_control.p_data, size );
        s_ep_control.p_data += size;
        s_ep_control.size -= size;
        MXC_USB->in_owner = (1<<ENDPOINT_CONTROL);
    }
    usb_ack(ENDPOINT_CONTROL);
}

static void read_control_out( void )
{
    // receiving control data from the host

    uint32_t dest_size = s_ep_control.size;
    uint32_t available = sie_ctrl_out->size;

    sie_ctrl_out->size = MXC_USB_MAX_PACKET;  // reset for subsequent reads

    uint32_t size = MIN(available,dest_size);
    memcpy( s_ep_control.p_data, s_ep_control.buffer, size );

    s_ep_control.p_data += size;
    s_ep_control.size -= size;

    if( s_ep_control.size )
        MXC_USB->out_owner = (1<<ENDPOINT_CONTROL);    // return buffer to usb sie control
    usb_ack( ENDPOINT_CONTROL );
}

static void read_bulk_out( void )
{
    // receiving serial data from the host
    uint32_t available;

    available = sie_bulk_out->size;    // number of bytes available to read
    s_read_overflow += (available - cbuf_write( s_cbuf_read, s_ep_out.buffer, available));
    ready_bulk_out();
    usb_ack( ENDPOINT_ACM_BULK_OUT );
}

static void write_control_in_begin( const void *pv_data, uint8_t size )
{
    // send control data to the host
    s_ep_control.size = size;
    s_ep_control.p_data = (void*)pv_data;
    write_control_in();
}

static void read_control_out_begin( const void *pv_data, uint8_t size )
{
    // prepare to receive control data from the host
    s_ep_control.size = size;
    s_ep_control.p_data = (void*)pv_data;
    MXC_USB->out_owner = (1<<ENDPOINT_CONTROL);
}

static void setup_standard_device_request( const usb_setup_t * p )
{
    switch( p->bRequest )
    {
        case SETUP_REQUEST_GET_DESCRIPTOR:
        {
            uint8_t type = p->wValue >> 8;
            switch( type )
            {
                case DESCRIPTOR_TYPE_DEVICE:
                {
                    DBG(("DESCRIPTOR_TYPE_DEVICE\n"));
                    write_control_in_begin( &s_descriptors.device, MIN(sizeof(s_descriptors.device),p->wLength) );
                    return;
                }
                case DESCRIPTOR_TYPE_CONFIGURATION:
                {
                    DBG(("DESCRIPTOR_TYPE_CONFIGURATION\n"));
                    write_control_in_begin( &s_descriptors.configuration, MIN(sizeof(s_descriptors.configuration),p->wLength) );
                    return;
                }
                case DESCRIPTOR_TYPE_STRING:
                {
                    DBG(("DESCRIPTOR_TYPE_STRING\n"));
                    static const uint8_t * s_strings[3] =
                    {
                        s_lang_id_desc,
                        s_mfg_id_desc,
                        s_prod_id_desc
                    };
                    uint8_t ndx = p->wValue & 0xFF;
                    if( ndx < ARRAY_COUNT(s_strings) )
                    {
                        write_control_in_begin( s_strings[ndx], MIN(s_strings[ndx][0],p->wLength) );
                        return;
                    }
                    break;
                }
            }
            break;
        }
        case SETUP_REQUEST_SET_CONFIGURATION:
        {
            usb_ack(ENDPOINT_CONTROL);
            return;
        }
    }
    usb_stall(ENDPOINT_CONTROL);
}

static void setup_standard_interface_request( const usb_setup_t * p )
{
    switch( p->bRequest )
    {
        case SETUP_REQUEST_GET_INTERFACE:
        {
            uint8_t zero = 0;
            DBG(("SETUP_REQUEST_GET_INTERFACE\n"));
            write_control_in_begin( &zero, sizeof(zero) );
            return;
        }
        case SETUP_REQUEST_SET_INTERFACE:
        {
            DBG(("SETUP_REQUEST_SET_INTERFACE\n"));
            usb_ack(ENDPOINT_CONTROL);
            return;
        }
    }
    usb_stall(ENDPOINT_CONTROL);
}

static void setup_class_interface_request( const usb_setup_t * p )
{
    static acm_line_coding_t line_coding =
    {
        .dwDTERate = 115200,
        .bDataBits = 8,
        .bParityType = 0,
        .bCharFormat = 0
    };

    switch( p->bRequest )
    {
        case ACM_SET_LINE_CODING:
        {
            DBG(("ACM_SET_LINE_CODING\n"));
            read_control_out_begin( &line_coding, sizeof(line_coding) );
            return;
        }
        case ACM_GET_LINE_CODING:
        {
            DBG(("ACM_GET_LINE_CODING\n"));
            write_control_in_begin( &line_coding, sizeof(line_coding) );
            return;
        }
    }
    usb_stall(ENDPOINT_CONTROL);
}

static void setup( void )
{
    usb_setup_reg_t setup;
    const usb_setup_t *p;
    {
        // copy setup payload into RAM so as to avoid non 32-bit access bus faults
        const usb_setup_reg_t *p_reg = (const usb_setup_reg_t*)&MXC_USB->setup0;
        setup.reg[0] = p_reg->reg[0];
        setup.reg[1] = p_reg->reg[1];
        p = &setup.setup;
    }
    switch( p->bmRequestType.type )
    {
        case SETUP_REQUEST_TYPE_STANDARD:
        {
            switch( p->bmRequestType.recipient )
            {
                case SETUP_REQUEST_TYPE_RECIPIENT_DEVICE:
                {
                    DBG(("SETUP_REQUEST_TYPE_RECIPIENT_DEVICE\n"));
                    setup_standard_device_request(p);
                    return;
                }
                case SETUP_REQUEST_TYPE_RECIPIENT_INTERFACE:
                {
                    DBG(("SETUP_REQUEST_TYPE_RECIPIENT_INTERFACE\n"));
                    setup_standard_interface_request(p);
                    return;
                }
            }
            break;
        }
        case SETUP_REQUEST_TYPE_CLASS:
        {
            DBG(("SETUP_REQUEST_TYPE_CLASS\n"));
            setup_class_interface_request(p);
            return;
        }
    }
    usb_stall(ENDPOINT_CONTROL);
}


void USB_IRQHandler( void )
{
    uint32_t intfl = MXC_USB->dev_intfl & MXC_USB->dev_inten;
    if( intfl & MXC_F_USB_DEV_INTFL_EP_IN )
    {
        uint32_t in_int = MXC_USB->in_int;
        if( in_int & (1<<ENDPOINT_ACM_BULK_IN) )
            write_bulk_in();
        if( in_int & (1<<ENDPOINT_CONTROL) )
            write_control_in();
        MXC_USB->in_int = in_int;
    }
    if( intfl & MXC_F_USB_DEV_INTFL_EP_OUT )
    {
        uint32_t out_int = MXC_USB->out_int;
        if( out_int & (1<<ENDPOINT_ACM_BULK_OUT) )
            read_bulk_out();
        if( out_int & (1<<ENDPOINT_CONTROL) )
            read_control_out();
        MXC_USB->out_int = out_int;
    }
    if( intfl & MXC_F_USB_DEV_INTFL_SETUP )
    {
        DBG(("MXC_F_USB_DEV_INTFL_SETUP\n"));
        setup();
    }
    MXC_USB->dev_intfl = intfl;
}
