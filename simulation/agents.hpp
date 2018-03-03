#ifndef INCLUDE_AGENTS_HPP
#define INCLUDE_AGENTS_HPP

#include "allocore/io/al_App.hpp"
#include "allocore/math/al_Quat.hpp"
#include "allocore/spatial/al_Pose.hpp"
#include "helper.hpp"
#include "agent_base.hpp"
#include "locations.hpp"

using namespace al;
using namespace std;

// struct Resource;
// struct Factory;
// struct Natural_Resource_Point;
// struct MetroBuilding;

struct Miner : Agent {
    int mesh_Nv;
    Vec3f movingTarget;
    Vec3f temp_pos;

    bool resourcePointFound;
    float distToClosestNRP;
    float distToClosestResource;
    int id_ClosestNRP;
    int id_ClosestResource;
    float searchResourceForce;
    float collectResourceForce;
    float sensitivityNRP;
    float sensitivityResource;
    float desireLevel;
    float separateForce;
    float resourceHoldings;

    bool overload;
    Miner(){
        maxAcceleration = 2;
        mass = 1.0;
        maxspeed = 0.5;
        minspeed = 0.1;
        maxforce = 0.05;
        initialRadius = 5;
        target_senseRadius = 10.0;
        desiredseparation = 3.0f;
        acceleration = Vec3f(0,0,0);
        velocity = Vec3f(0,0,0);
        pose.pos() = r();
        temp_pos = pose.pos();
        pose.pos() = pose.pos() * (NaturalRadius - FactoryRadius) + temp_pos.normalize(FactoryRadius + CirclePadding * 2.0);
        bioClock = 0;
        movingTarget = r();

        //relation to resource point
        distToClosestNRP = 120.0f;
        distToClosestResource = 120.0f;
        id_ClosestNRP = 0;
        resourcePointFound = false;
        id_ClosestResource = 0;
        searchResourceForce = 1.0;
        collectResourceForce = 1.0;
        sensitivityNRP = 30.0;
        sensitivityResource = 8.0;
        overload = false;

        //capitals
        resourceHoldings = 0.0;
        moneySavings = 30.0;
        poetryHoldings = 0.0;

        //social behaviors
        desireLevel = 0.1;
        separateForce = 1.5;

        //draw body
        scaleFactor = 0.3;
        mesh_Nv = addCone(body, moneySavings / 30.0, Vec3f(0,0,moneySavings / 30.0 * 3));
        for(int i=0; i<mesh_Nv; ++i){
			float f = float(i)/mesh_Nv;
			body.color(HSV(f*0.05,0.9,1));
		}
        body.decompress();
        body.generateNormals();
    }

    void run(vector<Natural_Resource_Point>& nrps, vector<Miner>& others){
        if (!overload){
            if (distToClosestNRP < 5.0) {
                resourcePointFound = true;
                collectResourceForce = 1.0;
                searchResourceForce = 0.1;
                desireLevel = 0.1;
                separateForce = 1.5;
            } else {
                resourcePointFound = false;
                collectResourceForce = 0.1;
                searchResourceForce = 1.0;
                separateForce = 0.3;
                desireLevel = 0.5;
                facingToward(movingTarget);
            }
            //resource mining
            seekResourcePoint(nrps);
            if (resourcePointFound == true){
                collectResource(nrps);
            } 
        } else {
            //find capitalist for a trade
        }
        
        //separate
        Vec3f sep(separate(others));
        sep *= separateForce;
        applyForce(sep);       
        // cout << resourcePointFound << "found??" << endl;
        // cout << distToClosestNRP << endl;
        // cout<< id_ClosestNRP << "  NRP" << endl;
        // cout << id_ClosestResource << " resource" << endl;

        //default behaviors
        inherentDesire(desireLevel, NaturalRadius);
        borderDetect();
        update();
    }


    void collectResource(vector<Natural_Resource_Point>& nrps){
        float min = 9999;
        int min_id = 0;
        if (!nrps[id_ClosestNRP].drained()){
            for (int i = nrps[id_ClosestNRP].resources.size() - 1; i >= 0; i--){
                if (!nrps[id_ClosestNRP].resources[i].isPicked){
                    Vec3f dist_difference = pose.pos() - nrps[id_ClosestNRP].resources[i].position;
                    double dist = dist_difference.mag();
                    if (dist < min){
                        min = dist;
                        min_id = i;
                        id_ClosestResource = min_id;
                        distToClosestResource = dist;
                    }
                }
            }
            if (distToClosestResource < sensitivityResource){
                if (!nrps[id_ClosestNRP].drained()){
                    Vec3f collectNR(seek(nrps[id_ClosestNRP].resources[min_id].position));
                    collectNR *= collectResourceForce;
                    applyForce(collectNR);
                    facingToward(nrps[id_ClosestNRP].resources[min_id].position);
                }
            }
        } else {
            resourcePointFound = false;
        }
    }
    void seekResourcePoint(vector<Natural_Resource_Point>& nrps){
        float min = 999;
        int min_id = 0;     
        for (int i = 0; i < nrps.size(); i++){
            if (!nrps[i].drained()){
                Vec3f dist_difference = pose.pos() - nrps[i].position;
                float dist = dist_difference.mag();
                if (dist < min){
                    min = dist;
                    min_id = i;
                    //update universal variable for other functions to use
                    distToClosestNRP = min;
                    id_ClosestNRP = min_id;
                }
            }
        }
        if (distToClosestNRP < sensitivityNRP){
            if (!nrps[min_id].drained()){
                resourcePointFound = true;
                Vec3f skNRP(seek(nrps[min_id].position));
                skNRP *= searchResourceForce;
                applyForce(skNRP);
                facingToward(nrps[min_id].position);
            } else {
                resourcePointFound = false;
            }
        } else {
            resourcePointFound = false;
        }
    }

    Vec3f separate(vector<Miner>& others){
        Vec3f sum;
        int count = 0;
        for (Miner m : others){
            Vec3f difference = pose.pos() - m.pose.pos();
            float d = difference.mag();
            if ((d > 0) && (d<desiredseparation)){
                Vec3f diff = difference.normalize();
                sum += diff;
                count ++;
            }
        }
        if (count > 0){
            sum /= count;
            sum.mag(maxspeed);
            Vec3f steer = sum - velocity;
            if (steer.mag() > maxforce){
                steer.normalize(maxforce);
            }
            return steer;
        } else {
            return Vec3f(0,0,0);
        }
    }

    void findPoems(){
        //30% probability
        if (rnd::prob(0.3)) {
            //find poems
        };
    }
    
    void draw(Graphics& g){
        g.pushMatrix();
        g.translate(pose.pos());
        g.rotate(pose.quat());
        g.scale(scaleFactor);
        g.draw(body);
        g.popMatrix();
    }

};

struct Worker : Agent {
    
    Worker(){

    }

    void seekFactory(const vector<Factory>& f){

    }

    void findPoems(){
        //30% probability
        if (rnd::prob(0.3)) {
            //find poems
        };
    }


};

struct Capitalist : Agent {
    int mesh_Nv;
    Vec3f movingTarget;

    Capitalist(){
        //initial params
        maxAcceleration = 4;
        mass = 1.0;
        maxspeed = 1;
        minspeed = 0.1;
        maxforce = 0.1;
        initialRadius = 5;
        target_senseRadius = 10.0;
        desiredseparation = 3.0f;
        acceleration = Vec3f(0,0,0);
        velocity = Vec3f(0,0,0);
        pose.pos() = r() * initialRadius;
        bioClock = 0;
        movingTarget = r();

        //capitals
        moneySavings = 30.0;
        poetryHoldings = 0.0;

        //draw body
        scaleFactor = 0.3; //richness?
        mesh_Nv = addCone(body, moneySavings / 30.0, Vec3f(0,0,moneySavings / 30.0 * 3));
        for(int i=0; i<mesh_Nv; ++i){
			float f = float(i)/mesh_Nv;
			body.color(HSV(f*0.2,0.9,1));
		}
        body.decompress();
        body.generateNormals();
    }
    void run(vector<MetroBuilding>& mbs){
        //flocking behaviors
        Vec3f ahb(avoidHittingBuilding(mbs));
        ahb *= 0.8;
        applyForce(ahb);

        //default behaviors
        borderDetect();
        inherentDesire(0.5, MetroRadius);
        facingToward(movingTarget);
        update();
    }

    void learnPoems(){

    }

    Vec3f avoidHittingBuilding(vector<MetroBuilding>& mbs){
        Vec3f sum;
        int count = 0;
        for (MetroBuilding mb : mbs){
            Vec3f difference = pose.pos() - mb.position;
            float d = difference.mag();
            if ((d > 0) && (d < desiredseparation)){
                Vec3d diff = difference.normalize();
                sum += diff;
                count++;
            }
        }
        if (count > 0){
            sum /= count;
            sum.mag(maxspeed);
            Vec3f steer = sum - velocity;
            if (steer.mag() > maxforce) {
                steer.normalize(maxforce);
            }
            return steer;
        } else {
            return Vec3f(0,0,0);
        }
    }

    Vec3f arrive(Vec3f& target){
        Vec3f desired = target - pose.pos();
        float d = desired.mag();
        desired.normalize();
        if (d < target_senseRadius){
            float m = MapValue(d, 0, target_senseRadius, minspeed, maxspeed);
            desired *= m;
        } else {
            desired *= maxspeed;
        }
        Vec3f steer = desired - velocity;
        if (steer.mag() > maxforce){
            steer.normalize(maxforce);
        }
        return steer;
    }
    void draw(Graphics& g){
        g.pushMatrix();
        g.translate(pose.pos());
        g.rotate(pose.quat());
        g.scale(scaleFactor);
        g.draw(body);
        g.popMatrix();
    }
};

#include "locations.hpp"

#endif