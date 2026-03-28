#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <cmath>
#include <iomanip>

using namespace std;

#define M_PI 3.14159265358979323846

struct UAV {
    int id;
    float lat, lon, alt;
    float velocity, yaw, pitch;
};

int main() 
{
    int NUM_UAVS = 300; //adjust here to change the number of UAVs 
    int TOTAL_FRAMES = 1000;  
    float dt = 0.016f;        
    
    string filename = "input_data_300_03.csv";
    ofstream file(filename);

    if (!file.is_open()) {
        cerr << "Khong the tao file!" << endl;
        return 1;
    }

    
    file << "ID,     Lat,     Lon,     Alt,     Velocity,     Yaw,     Pitch\n";

   
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<float> rand_pos(0.0f, 1000.0f);
    uniform_real_distribution<float> rand_alt(100.0f, 500.0f); 
    uniform_real_distribution<float> rand_vel(10.0f, 25.0f);  
    uniform_real_distribution<float> rand_yaw(0.0f, 360.0f);
    
    
    uniform_real_distribution<float> rand_steering(-3.0f, 3.0f); 
    uniform_real_distribution<float> rand_accel(-1.0f, 1.0f);    

    vector<UAV> uavs(NUM_UAVS);

    
    for (int i = 0; i < NUM_UAVS; i++) {
        uavs[i].id = i + 1;
        uavs[i].lat = rand_pos(gen);
        uavs[i].lon = rand_pos(gen);
        uavs[i].alt = rand_alt(gen);
        uavs[i].velocity = rand_vel(gen);
        uavs[i].yaw = rand_yaw(gen);
        uavs[i].pitch = uniform_real_distribution<float>(-20.0f, 20.0f)(gen); 
    }

    cout << "Processing " << TOTAL_FRAMES << " frame for " << NUM_UAVS << " UAV..." << endl;

    for (int frame = 0; frame < TOTAL_FRAMES; frame++) {
        for (int i = 0; i < NUM_UAVS; i++) {
            
            
            file << fixed << setprecision(6)
                 << uavs[i].id << ","
                 << uavs[i].lat << "," 
                 << uavs[i].lon << "," 
                 << uavs[i].alt << "," 
                 << uavs[i].velocity << "," 
                 << uavs[i].yaw << "," 
                 << uavs[i].pitch << ","  << endl;

            float yaw_rad = uavs[i].yaw * M_PI / 180.0f;
            float pitch_rad = uavs[i].pitch * M_PI / 180.0f;

        
            float vz = uavs[i].velocity * sin(pitch_rad);
            float v_xy = uavs[i].velocity * cos(pitch_rad);
            float vx = v_xy * sin(yaw_rad);
            float vy = v_xy * cos(yaw_rad);

            
            uavs[i].lon += vx * dt;
            uavs[i].lat += vy * dt; 
            uavs[i].alt += vz * dt; 

            
            uavs[i].yaw += rand_steering(gen);
            uavs[i].pitch += rand_steering(gen) * 0.5f; 
            uavs[i].velocity += rand_accel(gen);

            if (uavs[i].velocity < 10.0f) uavs[i].velocity = 10.0f;
            if (uavs[i].velocity > 60.0f) uavs[i].velocity = 60.0f;
            if (uavs[i].pitch > 45.0f) uavs[i].pitch = 45.0f;
            if (uavs[i].pitch < -45.0f) uavs[i].pitch = -45.0f;
        }
    }

    file.close();
    cout << "Successed " << filename << endl;
    cout << "Total: " << NUM_UAVS * TOTAL_FRAMES << " lines." << endl;

    return 0;
}