#ifndef MESH_ERROR_H__
#define MESH_ERROR_H__



#define MESH_ERROR_BASE_NUM      (0x0)       
#define MESH_ERROR_SDM_BASE_NUM  (0x1000)    
#define MESH_ERROR_SOC_BASE_NUM  (0x2000)    
#define MESH_ERROR_STK_BASE_NUM  (0x3000)    


#define MESH_SUCCESS                           (MESH_ERROR_BASE_NUM + 0) 
#define MESH_ERROR_SVC_HANDLER_MISSING         (MESH_ERROR_BASE_NUM + 1)  
#define MESH_ERROR_SOFTDEVICE_NOT_ENABLED      (MESH_ERROR_BASE_NUM + 2)  
#define MESH_ERROR_INTERNAL                    (MESH_ERROR_BASE_NUM + 3)  
#define MESH_ERROR_NO_MEM                      (MESH_ERROR_BASE_NUM + 4)  
#define MESH_ERROR_NOT_FOUND                   (MESH_ERROR_BASE_NUM + 5)  
#define MESH_ERROR_NOT_SUPPORTED               (MESH_ERROR_BASE_NUM + 6)  
#define MESH_ERROR_INVALID_PARAM               (MESH_ERROR_BASE_NUM + 7)  
#define MESH_ERROR_INVALID_STATE               (MESH_ERROR_BASE_NUM + 8)  
#define MESH_ERROR_INVALID_LENGTH              (MESH_ERROR_BASE_NUM + 9)  
#define MESH_ERROR_INVALID_FLAGS               (MESH_ERROR_BASE_NUM + 10) 
#define MESH_ERROR_INVALID_DATA                (MESH_ERROR_BASE_NUM + 11) 
#define MESH_ERROR_DATA_SIZE                   (MESH_ERROR_BASE_NUM + 12) 
#define MESH_ERROR_TIMEOUT                     (MESH_ERROR_BASE_NUM + 13) 
#define MESH_ERROR_NULL                        (MESH_ERROR_BASE_NUM + 14) 
#define MESH_ERROR_FORBIDDEN                   (MESH_ERROR_BASE_NUM + 15) 
#define MESH_ERROR_INVALID_ADDR                (MESH_ERROR_BASE_NUM + 16) 
#define MESH_ERROR_BUSY                        (MESH_ERROR_BASE_NUM + 17) 
#define MESH_ERROR_CONN_COUNT                  (MESH_ERROR_BASE_NUM + 18) 
#define MESH_ERROR_RESOURCES                   (MESH_ERROR_BASE_NUM + 19) 


#endif // MESH_ERROR_H__


