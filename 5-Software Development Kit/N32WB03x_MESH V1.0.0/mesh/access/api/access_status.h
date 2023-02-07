#ifndef ACCESS_STATUS_H__
#define ACCESS_STATUS_H__


typedef enum
{
    ACCESS_STATUS_SUCCESS                            = 0x00, /**< Command successfully processed. */
    ACCESS_STATUS_INVALID_ADDRESS                    = 0x01, /**< The provided value is not a valid address in this context. */
    ACCESS_STATUS_INVALID_MODEL                      = 0x02, /**< The provided Model index is not valid in this Element. */
    ACCESS_STATUS_INVALID_APPKEY                     = 0x03, /**< The provided AppKey index is not valid for this node. */
    ACCESS_STATUS_INVALID_NETKEY                     = 0x04, /**< The provided NetKey index is not valid for this node. */
    ACCESS_STATUS_INSUFFICIENT_RESOURCES             = 0x05, /**< The node cannot store provided information due to insufficient resources on node. */
    ACCESS_STATUS_KEY_INDEX_ALREADY_STORED           = 0x06, /**< The key with given index is already stored in the node with a different value. */
    ACCESS_STATUS_NOT_A_PUBLISH_MODEL                = 0x07, /**< The referenced Model is not a Publish Model. */
    ACCESS_STATUS_NOT_A_SUBSCRIBE_MODEL              = 0x08, /**< The referenced Model is not a Subscribe Model. */
    ACCESS_STATUS_STORAGE_FAILURE                    = 0x09, /**< The node was not able to store new value in persistent storage . */
    ACCESS_STATUS_FEATURE_NOT_SUPPORTED              = 0x0A, /**< The feature is not supported in this node. */
    ACCESS_STATUS_CANNOT_UPDATE                      = 0x0B, /**< The requested update operation cannot be performed due to general constraints. */
    ACCESS_STATUS_CANNOT_REMOVE                      = 0x0C, /**< The requested delete operation cannot be performed due to general constraints. */
    ACCESS_STATUS_CANNOT_BIND                        = 0x0D, /**< The requested bind operation cannot be performed due to general constraints. */
    ACCESS_STATUS_TEMPORARILY_UNABLE_TO_CHANGE_STATE = 0x0E, /**< The requested operation temporarily cannot be performed due to internal state of the node. */
    ACCESS_STATUS_CANNOT_SET                         = 0x0F, /**< The requested set operation cannot be performed due to general constraints. */
    ACCESS_STATUS_UNSPECIFIED_ERROR                  = 0x10, /**< An error occurred that does not correspond to any error conditions defined for a given state. */
    ACCESS_STATUS_INVALID_BINDING                    = 0x11, /**< The NetKeyIndex and AppKeyIndex combination is not valid. */
} access_status_t;

#endif
