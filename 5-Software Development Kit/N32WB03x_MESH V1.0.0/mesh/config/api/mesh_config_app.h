#ifndef MESH_CONFIG_APP_H__
#define MESH_CONFIG_APP_H__


#define DEVICE_COMPANY_ID (ACCESS_COMPANY_ID_NORDIC)
#define DEVICE_PRODUCT_ID (0x0000)
#define DEVICE_VERSION_ID (0x0000)
#define ACCESS_DEFAULT_TTL (30)
#define ACCESS_MODEL_COUNT (3)
#define ACCESS_ELEMENT_COUNT (1)
#define ACCESS_SUBSCRIPTION_LIST_COUNT (ACCESS_MODEL_COUNT)
#define ACCESS_RELIABLE_TRANSFER_COUNT (ACCESS_MODEL_COUNT)
#define HEALTH_SERVER_ELEMENT_COUNT (1)
#define DSM_SUBNET_MAX                                  (4)
#define DSM_APP_MAX                                     (8)
#define DSM_DEVICE_MAX                                  (1)
#define DSM_VIRTUAL_ADDR_MAX                            (1)
#define DSM_NONVIRTUAL_ADDR_MAX                         (3)
#define MESH_FEATURE_PB_GATT_ENABLED                    (1)
#define MESH_FEATURE_GATT_PROXY_ENABLED                 (1)
#define MESH_FEATURE_FRIEND_ENABLED (1)


#endif /* MESH_CONFIG_APP_H__ */
