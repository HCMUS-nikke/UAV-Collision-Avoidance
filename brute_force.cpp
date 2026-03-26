#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <chrono>
using namespace std;
const float PI = 3.14159265358979323846f;
const float METERS_PER_DEGREE = 111320.0f;

struct Drone {
    int id;
    float latitude;
    float longitude;
    float altitude;
    float velocity;
    float yaw;
    float pitch;
    void move(float dt)
    {
        float yaw_rad = yaw * (PI / 180.0f);
        float pitch_rad = pitch * (PI / 180.0f);
        float v_z = velocity * sin(pitch_rad);
        float v_xy = velocity * cos(pitch_rad);
        float v_x = v_xy * cos(yaw_rad);
        float v_y = v_xy * sin(yaw_rad);
        altitude += v_z * dt;
        latitude += (v_x * dt) / METERS_PER_DEGREE;
        longitude += (v_y * dt) / METERS_PER_DEGREE;
    }
};

vector<Drone> loadDronesFromCSV(const string& filename)
{
    vector<Drone> drones;
    ifstream file(filename);
    if (!file.is_open())
    {
        cerr << "Error, cant open file" << filename << "\n";
        return drones;
    }
    string line;
    getline(file, line);
    while (getline(file, line))
    {
        stringstream ss(line);
        string token;
        vector<string> columns;

        while (getline(ss, token, ','))
        {
            columns.push_back(token);
        }

        try {
            Drone d;
            d.id = stoi(columns[0]);
            d.latitude = stof(columns[2]);
            d.longitude = stof(columns[3]);
            d.altitude = stof(columns[4]);
            d.velocity = stof(columns[5]);
            d.yaw = stof(columns[6]);
            d.pitch = stof(columns[7]);
            drones.push_back(d);
        } catch (...) {
            continue;
        }
    }
    file.close();
    return drones;
}

int main() {
    vector<Drone> all_drones = loadDronesFromCSV("uav_data.csv");
    if (all_drones.empty())
    {
        cout << "Error, no data found";
        return 1;
    }
    int simulation_time = 60;
    float dt = 1.0f;
    int total_collisions = 0;
    auto start_time = chrono::high_resolution_clock::now();
    for (int t = 1; t <= simulation_time; ++t)
    {
        int collisions_this_second = 0;

        for (size_t i = 0; i < all_drones.size(); ++i)
        {
            all_drones[i].move(dt);
        }

        for (size_t i = 0; i < all_drones.size(); ++i)
        {
            for (size_t j = i + 1; j < all_drones.size(); ++j)
            {
                float lat_diff = abs(all_drones[i].latitude - all_drones[j].latitude);
                float lon_diff = abs(all_drones[i].longitude - all_drones[j].longitude);
                float alt_diff = abs(all_drones[i].altitude - all_drones[j].altitude);
                if (lat_diff < 0.02f && lon_diff < 0.02f && alt_diff < 100.0f) total_collisions++;
            }
        }

    }
    auto end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
    cout << "Total Collisions Found : " << total_collisions << "\n";
    cout << "Time Taken (Brute Force): " << duration.count() << " milliseconds\n";
    return 0;
}
