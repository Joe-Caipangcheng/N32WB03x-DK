#ifndef PROV_PROVISIONEE_H__
#define PROV_PROVISIONEE_H__

#include <stdbool.h>
#include <stdint.h>

#include "mesh_prov.h"
#include "mesh.h"
#include "mesh_prov_bearer.h"


uint32_t prov_provisionee_listen(mesh_prov_ctx_t * p_ctx, prov_bearer_t * p_bearer, const char * URI, uint16_t oob_info_sources);



#endif

