#include "gattm_int.h"

uint8_t attm_att_set_permission(uint16_t handle, uint16_t perm, uint16_t ext_perm)
{
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint8_t status = attmdb_get_attribute(handle, &elmt);

    if(status == ATT_ERR_NO_ERROR)
    {
        if(   attmdb_uuid16_comp(&elmt, ATT_DECL_PRIMARY_SERVICE)
           || attmdb_uuid16_comp(&elmt, ATT_DECL_SECONDARY_SERVICE)
           || attmdb_uuid16_comp(&elmt, ATT_DECL_CHARACTERISTIC)
           || attmdb_uuid16_comp(&elmt, ATT_DECL_INCLUDE)
           || attmdb_uuid16_comp(&elmt, ATT_DESC_CHAR_EXT_PROPERTIES))
        {
            status = ATT_ERR_REQUEST_NOT_SUPPORTED;
        }
        else
        {
            struct attm_att_desc* att  = elmt.info.att;
            struct attm_elmt prev_elmt = ATT_ELEMT_INIT;

            // Check if database Hash updated due to characteristic update
            if(   (attmdb_get_attribute(handle - 1, &prev_elmt) == ATT_ERR_NO_ERROR)
               && attmdb_uuid16_comp(&prev_elmt, ATT_DECL_CHARACTERISTIC))
            {
                if(PERM_GET(att->perm, PROP) != PERM_GET(perm, PROP))
                {
                    // Database updated
                    gattm_db_updated();
                }
            }

            /* update attribute permissions */
            att->perm = perm;

            /* update attribute extended permissions */
            if(PERM_GET(ext_perm, EKS))
            {
                att->info.max_length |= PERM_MASK_EKS;
            }
            else
            {
                att->info.max_length &= ~PERM_MASK_EKS;
            }
        }
    }

    return (status);
}

uint8_t attm_att_set_value(uint16_t handle, att_size_t length, att_size_t offset, uint8_t* value)
{
    struct attm_elmt elmt = ATT_ELEMT_INIT;
    uint8_t status = attmdb_get_attribute(handle, &elmt);

    if(status == ATT_ERR_NO_ERROR)
    {
        // value can not be set for following parameters
        if(   attmdb_uuid16_comp(&elmt, ATT_DECL_PRIMARY_SERVICE)
           || attmdb_uuid16_comp(&elmt, ATT_DECL_SECONDARY_SERVICE)
           || attmdb_uuid16_comp(&elmt, ATT_DECL_CHARACTERISTIC)
           || attmdb_uuid16_comp(&elmt, ATT_DECL_INCLUDE)
           || attmdb_uuid16_comp(&elmt, ATT_DESC_CHAR_EXT_PROPERTIES)
           || attmdb_uuid16_comp(&elmt, ATT_DESC_CLIENT_CHAR_CFG)
           || attmdb_uuid16_comp(&elmt, ATT_DESC_SERVER_CHAR_CFG))
        {
            status = ATT_ERR_REQUEST_NOT_SUPPORTED;
        }
        else
        {
            // check if data present in attribute
            if(PERM_GET(elmt.info.att->info.max_length, RI) == 0)
            {
                struct attm_att_value* val = (struct attm_att_value*)
                        (((uint8_t*)elmt.info.att) + PERM_GET(elmt.info.att->info.offset, VAL_OFFSET));

                if((offset) > val->length)
                {
                    status = ATT_ERR_INVALID_OFFSET;
                }
                /* check if value length is not too big */
                if((length+offset) > val->max_length)
                {
                    status = ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
                }
                /* update attribute value + length */
                else
                {
                    // copy data
                    memcpy(&(val->value[offset]), value, length);
                    val->length = length;
                }
            }
            // attribute value cannot be set
            else
            {
                status = ATT_ERR_REQUEST_NOT_SUPPORTED;
            }
        }
    }

    return (status);
}

uint8_t attm_reserve_handle_range(uint16_t* start_hdl, uint8_t nb_att)
{
    struct gattm_svc_desc svc_desc;
    uint8_t status;

    // create a fake service description containing handle range to allocate
    svc_desc.start_hdl = *start_hdl;
    svc_desc.nb_att    = nb_att - 1;

    // use check service handle to verify if the handle range can be allocated
    status = attmdb_svc_check_hdl(&svc_desc);

    // update service start handle that should be used for service allocation
    *start_hdl = svc_desc.start_hdl;

    //return if service range can be allocated or not.
    return (status);
}

