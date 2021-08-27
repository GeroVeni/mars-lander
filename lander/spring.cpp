#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main() {

    // declare variables
    double m, k, x, v, tMax, dt, t, a;
    vector<double> tList, xList, vList;

    // mass, spring constant, initial position and velocity
    m = 1;
    k = 1;
    x = 0;
    v = 1;

    // simulation time and timestep
    tMax = 100;
    dt = 0.1;

    // Euler integration
    for (t = 0; t <= tMax; t = t + dt) {

        // append current state to trajectories
        tList.push_back(t);
        xList.push_back(x);
        vList.push_back(v);

        // calculate new position and velocity
        a = -k * x / m;
        x = x + dt * v;
        v = v + dt * a;

    }

    xList.clear();
    vList.clear();

    double xPrev;
    x = 0;
    v = 1;
    xList.push_back(x);
    x += v * dt;

    // Verlet integration
    for (t = 0; t <= tMax; t += dt)
    {
        // append current state to trajectories
        xList.push_back(x);
        vList.push_back(v);

        // calculate new position
        a = -k * x / m;
        double xNew = 2 * x - xPrev + dt * dt * a;
        v = (xNew - xPrev) / (2 * dt);
        xPrev = x;
        x = xNew;
    }

    // Write the trajectories to file
    ofstream fout;
    fout.open("trajectories.txt");
    if (fout) { // file opened successfully
        for (int i = 0; i < tList.size(); i = i + 1) {
            fout << tList[i] << ' ' << xList[i] << ' ' << vList[i] << endl;
        }
    } else { // file did not open successfully
        cout << "Could not open trajectory file for writing" << endl;
    }
}
