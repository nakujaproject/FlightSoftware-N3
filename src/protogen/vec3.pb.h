/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8-dev */

#ifndef PB_FLIGHT_VEC3_PB_H_INCLUDED
#define PB_FLIGHT_VEC3_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _flight_Vec3 {
    float x;
    float y;
    float z;
} flight_Vec3;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define flight_Vec3_init_default                 {0, 0, 0}
#define flight_Vec3_init_zero                    {0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define flight_Vec3_x_tag                        1
#define flight_Vec3_y_tag                        2
#define flight_Vec3_z_tag                        3

/* Struct field encoding specification for nanopb */
#define flight_Vec3_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    x,                 1) \
X(a, STATIC,   SINGULAR, FLOAT,    y,                 2) \
X(a, STATIC,   SINGULAR, FLOAT,    z,                 3)
#define flight_Vec3_CALLBACK NULL
#define flight_Vec3_DEFAULT NULL

extern const pb_msgdesc_t flight_Vec3_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define flight_Vec3_fields &flight_Vec3_msg

/* Maximum encoded size of messages (where known) */
#define flight_Vec3_size                         15

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
