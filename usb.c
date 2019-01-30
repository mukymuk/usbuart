#include "global.h"
#include "usb.h"

#include "usb_regs.h"
#include "max32620.h"

#include "pwrman_regs.h"
#include "clkman_regs.h"

#define USB_SETUP_REQUEST_TYPE_DIRECTION_HOST_TO_DEVICE 0
#define USB_SETUP_REQUEST_TYPE_DIRECTION_DEVICE_TO_HOST 1

#define USB_SETUP_REQUEST_TYPE_STANDARD   0
#define USB_SETUP_REQUEST_TYPE_CLASS      1
#define USB_SETUP_REQUEST_TYPE_VENDOR     2

#define USB_SETUP_REQUEST_TYPE_RECIPIENT_DEVICE       0
#define USB_SETUP_REQUEST_TYPE_RECIPIENT_INTERFACE    1
#define USB_SETUP_REQUEST_TYPE_RECIPIENT_ENDPOINT     2

typedef struct
{
    uint8_t     recipient : 5;  // USB_SETUP_REQUEST_TYPE_RECIPIENT_*
    uint8_t     type      : 2;  // USB_SETUP_REQUEST_TYPE_*
    uint8_t     direction : 1;  // USB_SETUP_REQUEST_TYPE_DIRECTION_*
}
usb_setup_request_type_t;

#define USB_SETUP_REQUEST_GET_STATUS            0
#define USB_SETUP_REQUEST_CLEAR_FEATURE         1
#define USB_SETUP_REQUEST_SET_FEATURE           3
#define USB_SETUP_REQUEST_SET_ADDRESS           5
#define USB_SETUP_REQUEST_GET_DESCRIPTOR        6
#define USB_SETUP_REQUEST_SET_DESCRIPTOR        7
#define USB_SETUP_REQUEST_GET_CONFIGURATION     8
#define USB_SETUP_REQUEST_SET_CONFIGURATION     9
#define USB_SETUP_REQUEST_GET_INTERFACE         0x0A
#define USB_SETUP_REQUEST_SET_INTERFACE         0x11
#define USB_SETUP_REQUEST_SYNC_FRAME            0x12

typedef struct
{
    usb_setup_request_type_t        bmRequestType;  // USB_SETUP_REQUEST_TYPE_*
    uint8_t                         bRequest;       // USB_SETUP_REQUEST_*
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

/* Bitmasks for the bit-field bmRequestType */
#define RT_DEV_TO_HOST            0x80

#define RT_TYPE_STD               0x00
#define RT_TYPE_CLASS             0x20
#define RT_TYPE_VENDOR            0x40
#define RT_TYPE_MASK              (RT_TYPE_STD|RT_TYPE_CLASS|RT_TYPE_VENDOR)

#define RT_RECIP_MASK             0x1f
#define RT_RECIP_DEVICE           0x00
#define RT_RECIP_IFACE            0x01
#define RT_RECIP_ENDP             0x02
#define RT_RECIP_OTHER            0x03

/* Standard Device Requests for bRequest */
#define SDR_GET_STATUS            0x00
#define SDR_CLEAR_FEATURE         0x01
#define SDR_SET_FEATURE           0x03
#define SDR_SET_ADDRESS           0x05
#define SDR_GET_DESCRIPTOR        0x06
#define SDR_SET_DESCRIPTOR        0x07
#define SDR_GET_CONFIG            0x08
#define SDR_SET_CONFIG            0x09
#define SDR_GET_INTERFACE         0x0a
#define SDR_SET_INTERFACE         0x0b
#define SDR_SYNCH_FRAME           0x0c

/* Descriptor types for *_DESCRIPTOR */
#define DESC_DEVICE               1
#define DESC_CONFIG               2
#define DESC_STRING               3
#define DESC_INTERFACE            4
#define DESC_ENDPOINT             5
#define DESC_DEVICE_QUAL          6
#define DESC_OTHER_SPEED          7
#define DESC_IFACE_PWR            8

/* Feature types for *_FEATURE */
#define FEAT_ENDPOINT_HALT        0
#define FEAT_REMOTE_WAKE          1
#define FEAT_TEST_MODE            2

/* Get Status bit positions */
#define STATUS_EP_HALT            0x1
#define STATUS_DEV_SELF_POWERED   0x1
#define STATUS_DEV_REMOTE_WAKE    0x2

/* bmAttributes bit positions */
#define BMATT_REMOTE_WAKE         0x20
#define BMATT_SELF_POWERED        0x40

#define USB_EP_NUM_MASK   0x0F
#define USB_EP_DIR_MASK   0x80

#pragma pack(1)

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
usb_endpoint_buffer;

typedef struct
{
    usb_endpoint_buffer      out;
    usb_endpoint_buffer      in;
}
usb_endpoint0_buffer;

typedef struct
{
    usb_endpoint0_buffer    endpoint0;
    usb_endpoint_buffer     endpoint[3];
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
    uint16_t    self_powered : 1;
    uint16_t    remote_wakeup : 1;
    uint16_t    reserved : 14;
}
get_device_status_t;

#define USB_ENDPOINT_CONTROL 0

#pragma pack(4)

typedef struct
{
    usb_device_descriptor_t         device;
    usb_configuration_descriptor_t  configuration;
}
descriptors_t;

#pragma pack()

#define CDC_HEADER(length,subtype) { .bFunctionalLength = length, .bDescriptorType = 0x24, .bDescriptorSubtype = subtype }
#define DESC_HEADER(length,type)    { .bLength = length, .bDescriptorType = type }


static const __attribute__((aligned(4))) uint8_t s_lang_id_desc[] =
{
    0x04,         /* bLength */
    0x03,         /* bDescriptorType */
    0x09, 0x04    /* bString = wLANGID (see usb_20.pdf 9.6.7 String) */
};

static const __attribute__((aligned(4))) uint8_t s_mfg_id_desc[34] =
{
    34, 0x03,
    'M', 0,
    'a', 0,
    'x', 0,
    'i', 0,
    'm', 0,
    ' ', 0,
    'I', 0,
    'n', 0,
    't', 0,
    'e', 0,
    'g', 0,
    'r', 0,
    'a', 0,
    't', 0,
    'e', 0,
    'd', 0,
};

static const __attribute__((aligned(4))) uint8_t prod_id_desc[34] =
{
    34, 0x03,
    'M', 0,
    'A', 0,
    'X', 0,
    '3', 0,
    '2', 0,
    '6', 0,
    '2', 0,
    '0', 0,
    ' ', 0,
    'C', 0,
    'D', 0,
    'C', 0,
    '-', 0,
    'A', 0,
    'C', 0,
    'M', 0,
};

static const __attribute__((aligned(4))) descriptors_t s_descriptors =
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
            .bmCapabilities = 3,
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
            .bEndpointAddress = 0x83,
            .bmAttributes = 3,
            .wMaxPacketSize = MXC_USB_MAX_PACKET,
            .bInterval = 0xFF
        },
        .data_if =
        {
            .header = DESC_HEADER( sizeof(s_descriptors.configuration.data_if), DESC_INTERFACE ),
            .bInterfaceNumber = 0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 2,
            .bInterfaceClass = 10,
            .bInterfaceSubClass = 2,
            .bInterfaceProtocol = 0,
            .iInterface = 0
        },
        .ep_bulk_out =
        {
            .header = DESC_HEADER(sizeof(s_descriptors.configuration.ep_bulk_out),DESC_ENDPOINT),
            .bEndpointAddress = 1,
            .bmAttributes = 2,
            .wMaxPacketSize = MXC_USB_MAX_PACKET,
            .bInterval = 0
        },
        .ep_bulk_in =
        {
            .header = DESC_HEADER(sizeof(s_descriptors.configuration.ep_bulk_in),DESC_ENDPOINT),
            .bEndpointAddress = 0x82,
            .bmAttributes = 2,
            .wMaxPacketSize = MXC_USB_MAX_PACKET,
            .bInterval = 0
        },
    }
};

static __attribute__ ((aligned (512))) usb_sie_t s_usb_sie;


void usb_init( void )
{
    MXC_CLKMAN->clk_ctrl = MXC_F_CLKMAN_CLK_CTRL_USB_CLOCK_ENABLE;
    MXC_PWRMAN->pwr_rst_ctrl |= MXC_F_PWRMAN_PWR_RST_CTRL_USB_POWERED;
    MXC_USB->cn = 1;
    MXC_USB->ep_base = (uint32_t)&s_usb_sie;
    MXC_USB->dev_inten = MXC_F_USB_DEV_INTEN_SETUP | MXC_F_USB_DEV_INTEN_BRST | MXC_F_USB_DEV_INTEN_EP_IN | MXC_F_USB_DEV_INTEN_EP_OUT;
    MXC_USB->ep[USB_ENDPOINT_CONTROL] |= MXC_F_USB_EP_INT_EN;
    MXC_USB->dev_cn = (MXC_F_USB_DEV_CN_CONNECT | MXC_F_USB_DEV_CN_FIFO_MODE);
    NVIC_EnableIRQ(USB_IRQn);
}

static void wake( void )
{
    MXC_PWRMAN->pwr_rst_ctrl |= MXC_F_PWRMAN_PWR_RST_CTRL_USB_POWERED;
    MXC_USB->dev_cn &= ~MXC_F_USB_DEV_CN_ULPM;
}


static void usb_write( uint8_t ep, const void *p_data, uint8_t size )
{
    if( ep )
    {
        s_usb_sie.endpoint[ep-1].buffer[0].address = (uint32_t)p_data;
        s_usb_sie.endpoint[ep-1].buffer[0].size = size;
    }
    else
    {
        s_usb_sie.endpoint0.in.buffer[0].address = (uint32_t)p_data;
        s_usb_sie.endpoint0.in.buffer[0].size = size;
//        s_usb_sie.endpoint0.in.buffer[1].address = (uint32_t)p_data;
//        s_usb_sie.endpoint0.in.buffer[1].size = size;

//        s_usb_sie.endpoint0.out.buffer[0].address = (uint32_t)p_data;
//        s_usb_sie.endpoint0.out.buffer[0].size = size;
//        s_usb_sie.endpoint0.out.buffer[1].address = (uint32_t)p_data;
//        s_usb_sie.endpoint0.out.buffer[1].size = size;


    }
    MXC_USB->in_owner = (1<<ep);
}

static void inline usb_stall( uint8_t ep )
{
    MXC_USB->ep[ep] |= MXC_F_USB_EP_ST_STALL;
}

static void inline usb_ack( uint8_t ep )
{
    MXC_USB->ep[ep] |= MXC_F_USB_EP_ST_ACK;
}

#define USB_CONFIG_SELF_POWERED 0

typedef struct
{
    uint32_t remote_wakeup : 1;
    uint8_t configuration;
}
usb_t;

static usb_t s_usb;

#define USB_SETUP_REQUEST_FEATURE_DEVICE_REMOTE_WAKEUP  1
#define USB_SETUP_REQUEST_FEATURE_ENDPOINT_HALT         0
#define USB_SETUP_REQUEST_FEATURE_TEST_MODE             2

static void setup_device_request( const usb_setup_t * p )
{
    switch( p->bRequest )
    {
        case USB_SETUP_REQUEST_GET_STATUS:
        {
            get_device_status_t status;
            status.self_powered = USB_CONFIG_SELF_POWERED;
            status.remote_wakeup = s_usb.remote_wakeup;
            usb_write( USB_ENDPOINT_CONTROL, &status, sizeof(status) );
            return;
        }
        case USB_SETUP_REQUEST_CLEAR_FEATURE:
        {
            if( p->wValue == USB_SETUP_REQUEST_FEATURE_DEVICE_REMOTE_WAKEUP )
            {
                s_usb.remote_wakeup = false;
            }
            usb_ack(USB_ENDPOINT_CONTROL);
            return;
        }
        case USB_SETUP_REQUEST_SET_FEATURE:
        {
            if( p->wValue == USB_SETUP_REQUEST_FEATURE_DEVICE_REMOTE_WAKEUP )
            {
                s_usb.remote_wakeup = true;
            }
            usb_ack(USB_ENDPOINT_CONTROL);
            return;
        }
        case USB_SETUP_REQUEST_SET_ADDRESS:
        {
            usb_ack(USB_ENDPOINT_CONTROL);
            return;
        }
#define USB_DESCRIPTOR_TYPE_DEVICE			1
#define USB_DESCRIPTOR_TYPE_CONFIGURATION	2
#define USB_DESCRIPTOR_TYPE_STRING			3

        case USB_SETUP_REQUEST_GET_DESCRIPTOR:
        {
			switch( p->wValue & 0xFF )
			{
				case USB_DESCRIPTOR_TYPE_DEVICE:
				{
					usb_write( USB_ENDPOINT_CONTROL, &s_descriptors.device, sizeof(s_descriptors.device) );
					return;
				}
				case USB_DESCRIPTOR_TYPE_CONFIGURATION:
				{
					usb_write( USB_ENDPOINT_CONTROL, &s_descriptors.configuration, sizeof(s_descriptors.configuration) );
				}
				case USB_DESCRIPTOR_TYPE_STRING:
				{
				}
			}
            return;
        }
        case USB_SETUP_REQUEST_GET_CONFIGURATION:
        {
            usb_write( USB_ENDPOINT_CONTROL, &s_usb.configuration, sizeof(s_usb.configuration) );
            return;
        }
        case USB_SETUP_REQUEST_SET_CONFIGURATION:
        {
            s_usb.configuration = p->wValue;
            usb_ack(USB_ENDPOINT_CONTROL);
            return;
        }
    }
    usb_stall(USB_ENDPOINT_CONTROL);
}

static void setup_interface_request( const usb_setup_t * p )
{
    switch( p->bRequest )
    {
        case USB_SETUP_REQUEST_GET_STATUS:
        {
            uint16_t zero = 0;
            usb_write( USB_ENDPOINT_CONTROL, &zero, sizeof(zero) );
            return;
        }
        case USB_SETUP_REQUEST_GET_INTERFACE:
        {
            uint8_t zero = 0;
            usb_write( USB_ENDPOINT_CONTROL, &zero, sizeof(zero) );
            return;
        }
        case USB_SETUP_REQUEST_SET_INTERFACE:
        {
            usb_ack(USB_ENDPOINT_CONTROL);
            return;
        }
    }
    usb_stall(USB_ENDPOINT_CONTROL);
}

static void setup_endpoint_request( const usb_setup_t * p )
{
    switch( p->bRequest )
    {
        case USB_SETUP_REQUEST_GET_STATUS:
        {
            uint8_t zero = 0;
            usb_write( USB_ENDPOINT_CONTROL, &zero, sizeof(zero) );
            return;
        }
        case USB_SETUP_REQUEST_CLEAR_FEATURE:
        case USB_SETUP_REQUEST_SET_FEATURE:
        case USB_SETUP_REQUEST_SYNC_FRAME:
        {
            break;
        }
    }
    usb_stall(USB_ENDPOINT_CONTROL);
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
        case USB_SETUP_REQUEST_TYPE_STANDARD:
        {
            switch( p->bmRequestType.recipient )
            {
                case USB_SETUP_REQUEST_TYPE_RECIPIENT_DEVICE:
                {
                    setup_device_request(p);
                    break;
                }
                case USB_SETUP_REQUEST_TYPE_RECIPIENT_INTERFACE:
                {
                    setup_interface_request(p);
                    break;
                }
                case USB_SETUP_REQUEST_TYPE_RECIPIENT_ENDPOINT:
                {
                    setup_endpoint_request(p);
                    break;
                }
            }
            break;
        }
    }

}

void USB_IRQHandler( void )
{
    uint32_t intfl = MXC_USB->dev_intfl & MXC_USB->dev_inten;

    if( intfl & MXC_F_USB_DEV_INTFL_EP_IN )
    {
        usb_ack(USB_ENDPOINT_CONTROL);
    }
    if( intfl & MXC_F_USB_DEV_INTFL_EP_OUT )
    {
    }
    if( intfl & MXC_F_USB_DEV_INTFL_SETUP )
    {
        setup();
    }
    if( intfl & MXC_F_USB_DEV_INTFL_SUSP )
    {
        MXC_PWRMAN->pwr_rst_ctrl &= ~MXC_F_PWRMAN_PWR_RST_CTRL_USB_POWERED;
    }
    if( intfl & MXC_F_USB_DEV_INTFL_BRST )
    {
        // bus reset
        wake();
    }
    if( intfl & MXC_F_USB_DEV_INTFL_BRST_DN )
    {
        // bus reset complete
    }
    if( intfl & MXC_F_USB_DEV_INTFL_BACT )
    {
        // SYNC received
    }
    if( intfl & MXC_F_USB_DEV_INTFL_RWU_DN )
    {
        // remote wake-up done
    }
    if( intfl & MXC_F_USB_DEV_INTFL_DPACT )
    {
        // D+ activity
        wake();
    }
    MXC_USB->dev_intfl = intfl;
}
