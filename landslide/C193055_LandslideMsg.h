#ifndef C193055_LANDSLIDEMSG_H_
#define C193055_LANDSLIDEMSG_H_

#include <omnetpp.h>

using namespace omnetpp;

class C193055_LandslideMsg : public cMessage
{
  private:
    double rainfall; // Rainfall (mm)
    double ema10;    // EMA10 value
    int towerID;     // Destination tower ID

  public:
    C193055_LandslideMsg(const char *name = nullptr) : cMessage(name),
         rainfall(0.0), ema10(0.0), towerID(-1) {}

    // Getters and setters
    void setRainfall(double r) { rainfall = r; }
    double getRainfall() const { return rainfall; }

    void setEMA10(double e) { ema10 = e; }
    double getEMA10() const { return ema10; }

    void setTowerID(int id) { towerID = id; }
    int getTowerID() const { return towerID; }

    // Override duplicate method
    virtual C193055_LandslideMsg *dup() const override { return new C193055_LandslideMsg(*this); }
};

#endif /* C193055_LANDSLIDEMSG_H_ */
