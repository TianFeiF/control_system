#include <iostream>
#include <vector>
#include <cstring>
#include <cassert>
#include "test_all.hpp"

// Forward declaration of the function to test
int load_axis_config(const char* config_path, slave_data* slaves, int max_slaves);

void print_axis_info(const slave_data& slave) {
    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "Axis ID: " << slave.config.axis_id << std::endl;
    std::cout << "Name: " << slave.config.name << std::endl;
    std::cout << "Description: " << slave.config.description << std::endl;
    
    std::cout << "[Mechanical]" << std::endl;
    std::cout << "  Resolution: " << slave.config.mechanical.encoder_resolution_bits << " bits" << std::endl;
    std::cout << "  Gear Ratio: " << slave.config.mechanical.gear_ratio << std::endl;
    std::cout << "  Unit/Rev: " << slave.config.mechanical.unit_per_rev << std::endl;
    
    std::cout << "[Kinematics]" << std::endl;
    std::cout << "  Max Vel: " << slave.config.kinematics.max_velocity_units << std::endl;
    std::cout << "  Max Acc: " << slave.config.kinematics.max_acceleration_units << std::endl;
    
    std::cout << "[Limits]" << std::endl;
    std::cout << "  Soft Limit (+): " << slave.config.limits.soft_limit_pos_units << std::endl;
    std::cout << "  Soft Limit (-): " << slave.config.limits.soft_limit_neg_units << std::endl;
    
    std::cout << "[Synchronization]" << std::endl;
    std::cout << "  Synced: " << (slave.config.synchronization.is_synced ? "Yes" : "No") << std::endl;
    if (slave.config.synchronization.is_synced) {
        std::cout << "  Master ID: " << slave.config.synchronization.sync_master_axis_id << std::endl;
        std::cout << "  Type: " << slave.config.synchronization.sync_type << std::endl;
    }
}

int main() {
    std::cout << "Starting Config Parser Test..." << std::endl;

    // Simulate an array of slaves. 
    // In real app, this matches SLAVE_COUNT or similar.
    const int TEST_SLAVE_COUNT = 10;
    slave_data slaves[TEST_SLAVE_COUNT];
    
    // Initialize with zeros to ensure we detect changes
    std::memset(slaves, 0, sizeof(slaves));

    const char* config_file = "/home/phi/control_system/test/config/axis_config.json";
    
    std::cout << "Loading config from: " << config_file << std::endl;
    int result = load_axis_config(config_file, slaves, TEST_SLAVE_COUNT);

    if (result != 0) {
        std::cerr << "Failed to load configuration!" << std::endl;
        return 1;
    }

    std::cout << "Configuration loaded successfully." << std::endl;

    // Validation
    // We expect Axis 1, 2, 3 to be populated based on the json created earlier.
    
    // Check Axis 1 (Index 0)
    assert(slaves[0].config.axis_id == 1);
    assert(strcmp(slaves[0].config.name, "X_Axis_Gantry_Master") == 0);
    assert(slaves[0].config.mechanical.gear_ratio == 15.12);
    assert(slaves[0].config.synchronization.is_synced == true);

    // Check Axis 2 (Index 1)
    assert(slaves[1].config.axis_id == 2);
    assert(strcmp(slaves[1].config.name, "X_Axis_Gantry_Slave") == 0);
    assert(slaves[1].config.synchronization.sync_master_axis_id == 1);
    assert(strcmp(slaves[1].config.synchronization.sync_type, "slave") == 0);

    // Check Axis 3 (Index 2)
    assert(slaves[2].config.axis_id == 3);
    assert(strcmp(slaves[2].config.name, "Y_Axis") == 0);
    assert(slaves[2].config.mechanical.encoder_resolution_bits == 17);

    // Print details for visual verification
    for (int i = 0; i < 9; ++i) {
        print_axis_info(slaves[i]);
    }

    std::cout << "\nTest Passed! All assertions verified." << std::endl;

    return 0;
}
