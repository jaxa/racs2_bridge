#ifndef _racs2_bridge_msg_h_
#define _racs2_bridge_msg_h_

/*
** Type definition (websocket data format for racs2 bridge)
*/
#define BRIDGE_HEADER_LNGTH       32
#define BODY_DATA_LENGTH_LNGTH     4
#define BODY_DATA_MAX_LNGTH      128
#define BODY_DATA_OFFSET          (BRIDGE_HEADER_LNGTH + BODY_DATA_LENGTH_LNGTH)

typedef struct
{
    uint8              header[BRIDGE_HEADER_LNGTH];
    uint8              body_data_length[BODY_DATA_LENGTH_LNGTH];
    uint8              body_data[BODY_DATA_MAX_LNGTH];

}   OS_PACK racs2_bridge_msg_t  ;

#define RACS2_BRIDGE_MSG_LNGTH   sizeof ( racs2_bridge_msg_t )

#endif /* _racs2_bridge_msg_h_ */