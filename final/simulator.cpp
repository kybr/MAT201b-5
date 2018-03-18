#include "allocore/io/al_App.hpp"
#include "allocore/math/al_Quat.hpp"
#include "allocore/spatial/al_Pose.hpp"

#include "helper.hpp"

#include "agent_managers.hpp"

#include "location_managers.hpp"

#include "common.hpp"

#include "Cuttlebone/Cuttlebone.hpp"
#include "alloutil/al_AlloSphereAudioSpatializer.hpp"
#include "alloutil/al_Simulator.hpp"

using namespace al;
using namespace std;

// Mengyu Chen, 2018
// mengyuchen@ucsb.edu

struct MyApp : App, AlloSphereAudioSpatializer, InterfaceServerClient {
  Material material;
  Light light;

  // initial location managers
  Metropolis metropolis;
  Factories factories;
  NaturalResourcePointsCollection
      NaturalResourcePts;  // manager for Natural Resource Points

  // initial agent managers
  Capitalist_Entity capitalists;
  Miner_Group miners;
  Worker_Union workers;

  // market manager
  MarketManager marketManager;

  // for cuttlebone
  State state;
  cuttlebone::Maker<State> maker;

  MyApp()
      : maker(Simulator::defaultBroadcastIP()),
        InterfaceServerClient(Simulator::defaultInterfaceServerIP()) {
    light.pos(0, 0, 0);   // place the light
    nav().pos(0, 0, 50);  // place the viewer //80
    lens().far(400);      // set the far clipping plane
    background(Color(0.4));
    initWindow();
    // initAudio(44100);

    // set speaker layout
    // const int numSpeakers = 2;
    // Speaker speakers[numSpeakers] = {
    //     Speaker(0, 45, 0),
    //     Speaker(1, -45, 0),
    // };

    // speakerLayout.addSpeaker(speakers[0]);
    // speakerLayout.addSpeaker(speakers[1]);

    // spat = new AmbisonicsSpatializer(speakerLayout, 2, 1);

    // scene.usePerSampleProcessing(false);
    // listener = scene()->createListener(spat);

    // allo audio
    AlloSphereAudioSpatializer::initAudio();
    AlloSphereAudioSpatializer::initSpatialization();
    for (unsigned i = 0; i < 15; ++i) {
      scene()->addSource(*capitalists.cs[i].soundSource);
    }
    scene()->usePerSampleProcessing(false);

    // generate factories according to number of capitalists
    factories.generate(capitalists);
    metropolis.generate(capitalists);
    marketManager.statsInit(capitalists, workers, miners);
    workers.initID();
  }
  void onAnimate(double dt) {
    // this updates your nav, especially you use a controller / joystick
    while (InterfaceServerClient::oscRecv().recv()) {
    }

    // market
    marketManager.populationMonitor(capitalists, workers, miners, factories.fs);
    marketManager.capitalMonitor(capitalists, workers, miners, factories.fs);
    marketManager.updatePrice(capitalists, workers, miners);

    // related to market
    factories.getLaborPrice(marketManager);
    miners.calculateResourceUnitPrice(factories.fs);

    // locations
    metropolis.run();
    factories.run(capitalists);
    NaturalResourcePts.run();

    // agents
    capitalists.run(metropolis.mbs);
    miners.run(NaturalResourcePts.nrps, miners.ms, capitalists.cs);
    workers.run(factories.fs, workers.workers, capitalists.cs);

    // interaction between groups
    NaturalResourcePts.checkMinerPick(miners.ms);
    factories.checkWorkerNum(workers.workers);
    metropolis.mapCapitalistStats(capitalists.cs);
    capitalists.getResource(miners.ms);
    capitalists.getWorkersPaymentStats(factories.fs);

    // pay workers
    factories.payWorkers(marketManager);

    // locational behaviors
    factories.drawLinks(capitalists);

    // debug
    // cout << workers[0].id_ClosestFactory << " i m heading to " << endl;
    // cout << workers[0].distToClosestFactory << " this much far " << endl;
    // cout << factories.fs[0].workersWorkingNum << " =  workers " << endl;
    // cout << "  " << endl;
    // cout << factories.fs[0].materialStocks << "fc material" <<endl;
    // cout << capitalists.cs[0].resourceHoldings << "cp resource" << endl;
    // cout << capitalists.cs[0].resourceClock << "cp clock" <<endl;
    // cout << nrps.nrps[0].drained() << " drained?" << endl;
    // cout << nrps.nrps[0].regeneration_rate << " regen rate" << endl;
    // cout << nrps.nrps[0].afterDrainTimer << " timer" << endl;
    // cout << nrps.nrps[0].resources[0].isPicked << "  r0 is picked?" << endl;
    // cout << nrps.nrps[0].resources[0].beingPicked << "  r0 being picked?" <<
    // endl; cout << nrps.nrps[0].pickCount << "  pickcount?" << endl; cout <<
    // nrps.nrps[0].resources.size() << "size = count = " <<
    // nrps.nrps[0].pickCount << endl;

    // for cuttlebone
    state.numMiners = miners.ms.size();
    state.numWorkers = workers.workers.size();
    state.numCapitalists = capitalists.cs.size();
    state.numResources = NaturalResourcePts.nrps.size() * 7;

    for (int i = 0; i < miners.ms.size(); i++) {
      state.miner_pose[i] = miners.ms[i].pose;
      state.miner_scale[i] = miners.ms[i].scaleFactor;
      state.miner_poetryHoldings[i] = miners.ms[i].poetryHoldings;
      state.miner_bankrupted[i] = miners.ms[i].bankrupted();
      state.miner_fullpack[i] = miners.ms[i].fullpack;
    }
    for (int i = 0; i < workers.workers.size(); i++) {
      state.worker_pose[i] = workers.workers[i].pose;
      state.worker_scale[i] = workers.workers[i].scaleFactor;
      state.worker_poetryHoldings[i] = workers.workers[i].poetryHoldings;
      state.worker_bankrupted[i] = workers.workers[i].bankrupted();
      state.worker_lines_posA[i] = workers.lines[i].vertices()[0];
      state.worker_lines_posB[i] = workers.lines[i].vertices()[1];
    }
    for (int i = 0; i < capitalists.cs.size(); i++) {
      state.capitalist_pose[i] = capitalists.cs[i].pose;
      state.capitalist_scale[i] = capitalists.cs[i].scaleFactor;
      state.capitalist_poetryHoldigs[i] = capitalists.cs[i].poetryHoldings;
      state.capitalist_bankrupted[i] = capitalists.cs[i].bankrupted();
      state.capitalist_lines_posA[i] = factories.lines[i].vertices()[0];
      state.capitalist_lines_posB[i] = factories.lines[i].vertices()[1];
      state.factory_pos[i] = factories.fs[i].position;
      state.factory_rotation_angle[i] = factories.fs[i].angle1;
      state.factory_facing_center[i] = factories.fs[i].facing_center;
      state.factory_size[i] = factories.fs[i].scaleFactor;
      state.factory_color[i] = factories.fs[i].c;
      state.building_pos[i] = metropolis.mbs[i].position;
      state.building_size[i] = metropolis.mbs[i].scaleFactor;
      state.building_scaleZ[i] = metropolis.mbs[i].scaleZvalue;
    }
    for (int i = 0; i < NaturalResourcePts.nrps.size(); i++) {
      state.resource_point_pos[i] = NaturalResourcePts.nrps[i].position;
      for (int j = 0; j < NaturalResourcePts.nrps[i].resources.size(); j++) {
        state
            .resource_pos[i * NaturalResourcePts.nrps[i].resources.size() + j] =
            NaturalResourcePts.nrps[i].resources[j].position;
        state.resource_angleA[i * NaturalResourcePts.nrps[i].resources.size() +
                              j] =
            NaturalResourcePts.nrps[i].resources[j].angle1;
        state.resource_angleB[i * NaturalResourcePts.nrps[i].resources.size() +
                              j] =
            NaturalResourcePts.nrps[i].resources[j].angle2;
        state.resource_scale[i * NaturalResourcePts.nrps[i].resources.size() +
                             j] =
            NaturalResourcePts.nrps[i].resources[j].scaleFactor;
        state.resource_picked[i * NaturalResourcePts.nrps[i].resources.size() +
                              j] =
            NaturalResourcePts.nrps[i].resources[j].isPicked;
      }
    }
    state.metro_rotate_angle = metropolis.angle;
    state.nav_pose = nav();

    maker.set(state);
  }
  void onDraw(Graphics& g) {
    material();
    light();
    // glEnable(GL_POINT_SPRITE);
    g.blendAdd();
    // draw all the entities
    metropolis.draw(g);
    factories.draw(g);
    NaturalResourcePts.draw(g);
    capitalists.draw(g);
    miners.draw(g);
    workers.draw(g);
  }
  virtual void onSound(AudioIOData& io) {
    gam::Sync::master().spu(AlloSphereAudioSpatializer::audioIO().fps());
    for (unsigned i = 0; i < 15; ++i) {
      capitalists.cs[i].updateAuidoPose();
      capitalists.cs[i].onProcess(io);
      io.frame(0);
    }
    //  listener()->pose(nav());
    //   scene()->render(io);
  }
  void onKeyDown(const ViewpointWindow&, const Keyboard& k) {
    switch (k.key()) {
      case '1':
        factories.drawingLinks = !factories.drawingLinks;
        break;
      case '2':
        miners.drawingLinks = !miners.drawingLinks;
        break;
      case '3':
        workers.drawingLinks = !workers.drawingLinks;
        break;
      case '4':
        break;
      case '0':
        nav().pos(0, 0, 80);
        nav().faceToward(Vec3f(0, 0, 0), 1);
    }
  }
};

int main() {
  MyApp app;
  app.AlloSphereAudioSpatializer::audioIO().start();
  app.InterfaceServerClient::connect();
  app.maker.start();
  app.start();
}
