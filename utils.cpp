//
// Created by Jackson Hall on 4/29/2020.
//

#include <utility>
#include <vector>
#include <cmath>

using namespace std;


double square(double base){
    return pow(base, 2);
}

struct point {
    vector<double> coords;
    
    virtual double distanceTo(vector<double> otherCoords) = 0;
};

struct point3d : point {
    double x, y, z;

    point3d() : point3d(0, 0, 0){
    }

    point3d(double x, double y, double z){
        this->x = x;
        this->y = y;
        this->z = z;
        coords.push_back(x);
        coords.push_back(y);
        coords.push_back(z);
    }

    double distanceTo(vector<double> otherCoords) override {
        if (otherCoords.size() == 3){
            // Regular 3d distance
            return sqrt(square(x - otherCoords[0])
                    + square(y - otherCoords[1])
                    + square(z - otherCoords[2]));
        } else if (otherCoords.size() == 4){
            // This 3d point has no distance into 4th dimension (a),
            // so don't calculate difference in that direction,
            // just use value from other coords
            return sqrt(square(x - otherCoords[0])
                    + square(y - otherCoords[1])
                    + square(z - otherCoords[2])
                    + square(otherCoords[3]));
        }
    }
};

struct point4d : point {
public:
    double x, y, z, a;

    point4d(){
        this->x = this->y = this->z = this->a = 0;
    }

    point4d(double x, double y, double z, double a){
        this->x = x;
        this->y = y;
        this->z = z;
        this->a = a;
    }

    double distanceTo(vector<double> otherCoords) override {
        if (otherCoords.size() == 3){
            // A 3d point has no distance into 4th dimension (a),
            // so don't calculate difference in a direction,
            // just use value from other coords
            return sqrt(square(x - otherCoords[0])
                    + square(y - otherCoords[1])
                    + square(z - otherCoords[2])
                    + square(a));
        } else if (otherCoords.size() == 4){
            // Regular 4d distance
            return sqrt(square(x - otherCoords[0])
                    + square(y - otherCoords[1])
                    + square(z - otherCoords[2])
                    + square(a - otherCoords[3]));
        } else {
            // Bad input
            return -1;
        }
    }
};

struct edge {
    point *p1, *p2;
    pair<point*, point*> endpoints;

    edge(point *p1, point *p2){
        this->p1 = (point *) p1;
        this->p2 = (point *) p2;
        endpoints = make_pair(p1, p2);
    }

    double length(){
        return p1->distanceTo(p2->coords);
    }
};

struct spatialVector {
    vector<double> components;

    spatialVector(vector<double> components){
        this->components = move(components);
    }

    double magnitude(){
        double sumOfSquares = 0;

        for (auto & component : components){
            sumOfSquares += square(component);
        }

        return sqrt(sumOfSquares);
    }
};