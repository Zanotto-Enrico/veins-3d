#ifndef STATISTICS_H_
#define STATISTICS_H_

#include <omnetpp.h>
#include <fstream>
#include <sstream>

#include "veins/SignalStats.h"

using namespace omnetpp;

class Statistics : public cSimpleModule {
  protected:
    std::string outputFilePath;
    std::ofstream file;
    int run;



    void handleSignal(cComponent* source, simsignal_t signalID, cObject* obj);
    virtual void initialize() override;
    virtual void finish() override;
    virtual void handleMessage(cMessage *msg) override;
  public:
  void writeDataToCSV(double src, double dst, double dst_posx, double dst_posy, double snr, double rssi, double sim_time, double h_bs, const std::string& module, SignalStats signalStats);
};

#endif /* STATISTICS_H_ */
