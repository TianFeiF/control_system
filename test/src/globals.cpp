#include "test_all.hpp"

// Define global variables here to avoid multiple definition errors
ec_master_t *master = NULL;
ec_domain_t *domain1 = NULL;
uint8_t *domain1_pd = NULL;
struct timespec wakeup_time;

slave_data device_io;
slave_data device_hcfa_servo[3];
slave_data device_hans_robot[3][2];
slave_data device_f2838x;
