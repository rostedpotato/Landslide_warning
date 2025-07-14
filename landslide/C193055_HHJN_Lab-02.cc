#include <string.h>
#include <omnetpp.h>
#include "C193055_LandslideMsg.h"
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace omnetpp;

// Enumeration untuk sensor state
enum SensorState { IDLE, ACTIVE_RTS, WAITING_CTS, ACTIVE_CTS };

// Struct untuk menyimpan data CSV
struct CSVRow {
    std::string datetime;
    double rainfall;
    double ema10;
    std::string predict;
};

class C193055_HHJN : public cSimpleModule
{
  private:
    cMessage *timerMsg;      // Timer untuk penjadwalan pengiriman
    cMessage *ctsTimer;      // Timer untuk CTS delay
    cMessage *syncTimer;     // Timer untuk periodic time sync
    int sensorID;            // ID sensor
    bool isMicrocontroller;  // Flag untuk node ESP32/mikrokontroler
    SensorState sensorState; // Sensor state: IDLE, ACTIVE_RTS, WAITING_CTS, ACTIVE_CTS
    simtime_t localTime;     // Local time for synchronization
    simtime_t timeOffset;    // Time offset from reference node
    bool isTimeSynced;       // Flag indicating if node is time synchronized
    simtime_t lastSyncTime;  // Last time synchronization was performed
    double driftRate;        // Clock drift rate
    static constexpr double MAX_DRIFT = 0.0001; // Maximum allowed drift rate
    bool isUserNode;         // Flag for user node
    bool isServer;           // Flag for server node

    // Shared CSV file pointer
    static std::ifstream globalCsvFile;
    static bool csvInitialized;
    static std::ofstream alertLogFile;
    static bool alertLogInitialized;

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    CSVRow getNextCSVRow();
    void logPropagationDetails();
    void sendTimeSync();
    void handleTimeSync(cMessage *msg);
    void updateLocalTime(simtime_t referenceTime);
    void performPairwiseSync(simtime_t t1, simtime_t t2, simtime_t t3, simtime_t t4);
    void handleRTS();
    void handleCTS();
    void logAlert(int sensorID, const std::string& datetime, double rainfall, double ema10, std::string predict);
    void sendAlertToUser(int sensorID, double rainfall, double ema10);
    void handleAlert(cMessage *msg);

  public:
    C193055_HHJN() : cSimpleModule() {
        alertLogInitialized = false;
    }
};

Define_Module(C193055_HHJN);

// Inisialisasi anggota static
std::ifstream C193055_HHJN::globalCsvFile;
bool C193055_HHJN::csvInitialized = false;
std::ofstream C193055_HHJN::alertLogFile;
bool C193055_HHJN::alertLogInitialized = false;

void C193055_HHJN::initialize()
{
    timerMsg = new cMessage("Timer");
    ctsTimer = new cMessage("CTSTimer");
    syncTimer = new cMessage("SyncTimer");
    sensorID = -1;
    isMicrocontroller = false;
    sensorState = IDLE;
    localTime = simTime();
    timeOffset = 0;
    isTimeSynced = false;
    lastSyncTime = 0;
    driftRate = uniform(-MAX_DRIFT, MAX_DRIFT);
    isUserNode = false;
    isServer = false;

    std::string csvFilename = "data_gabungan.csv";
    if (!csvInitialized) {
        EV_INFO << "[INIT] Attempting to open CSV file: " << csvFilename << endl;
        globalCsvFile.open(csvFilename);
        if (!globalCsvFile.is_open()) {
            EV_ERROR << "[INIT] Error: Cannot open CSV file: " << csvFilename << endl;
            EV_ERROR << "[INIT] Current working directory: " << cSimulation::getActiveSimulation()->getEnvir()->getConfigEx()->getVariable("cwd") << endl;
            EV_ERROR << "[INIT] Please ensure the CSV file exists in the simulation directory" << endl;
            endSimulation();
        }
        std::string header;
        std::getline(globalCsvFile, header);
        csvInitialized = true;
        EV_INFO << "[INIT] CSV file successfully opened" << endl;
    }

    if (!alertLogInitialized) {
        alertLogFile.open("landslide_alerts.csv");
        if (!alertLogFile.is_open()) {
            EV_ERROR << "[INIT] Error: Cannot open alert log file" << endl;
            endSimulation();
        }
        alertLogFile << "Time,SensorID,Rainfall,EMA10,Predict\n";
        alertLogInitialized = true;
    }

    if (strncmp("sensor", getName(), 6) == 0) {
        sscanf(getName(), "sensor%d", &sensorID);
        EV_INFO << "[INIT] Sensor node " << getName()
                << " (ID: " << sensorID << ") initialized" << endl;
        scheduleAt(simTime() + uniform(0, 1), timerMsg);
    }
    else if (strncmp("esp32", getName(), 5) == 0) {
        isMicrocontroller = true;
        EV_INFO << "[INIT] ESP32 node " << getName() << " initialized" << endl;
        // ESP32 nodes initiate time synchronization
        if (strcmp(getName(), "esp32_1") == 0) {
            scheduleAt(simTime() + 1, syncTimer);
        }
    }
    else if (strncmp("user", getName(), 4) == 0) {
        isUserNode = true;
        EV_INFO << "[INIT] User node initialized" << endl;
    }
    else if (strncmp("server", getName(), 6) == 0) {
        isServer = true;
        EV_INFO << "[INIT] Server node initialized" << endl;
    }
}

CSVRow C193055_HHJN::getNextCSVRow()
{
    CSVRow result;
    result.rainfall = 0.0;
    result.ema10 = 0.0;
    
    std::string line;
    if (std::getline(globalCsvFile, line)) {
        std::stringstream ss(line);
        std::string prcpStr, kejadian, lokasi, kabupaten, penyebab, sma10Str, ema10Str;
        std::getline(ss, result.datetime, ',');
        std::getline(ss, prcpStr, ',');
        std::getline(ss, kejadian, ',');
        std::getline(ss, lokasi, ',');
        std::getline(ss, kabupaten, ',');
        std::getline(ss, penyebab, ',');
        std::getline(ss, sma10Str, ',');
        std::getline(ss, ema10Str, ',');

        if (!prcpStr.empty()) {
            try {
                result.rainfall = std::stod(prcpStr);
            } catch (const std::exception& e) {
                EV_WARN << "[CSV] Invalid rainfall data at " << result.datetime
                        << " for sensor " << sensorID << ": '" << prcpStr << "'" << endl;
            }
        }

        if (!ema10Str.empty()) {
            try {
                result.ema10 = std::stod(ema10Str);
            } catch (const std::exception& e) {
                EV_WARN << "[CSV] Invalid EMA10 data at " << result.datetime
                        << " for sensor " << sensorID << ": '" << ema10Str << "'" << endl;
            }
        }

        EV_DEBUG << "[CSV] Data read: " << result.datetime
                 << " | Rainfall: " << std::fixed << std::setprecision(2) << result.rainfall
                 << " | EMA10: " << result.ema10 << endl;
    }
    return result;
}

void C193055_HHJN::logPropagationDetails()
{
    // Get position of this node and its parent ESP32
    cModule *parentESP32 = nullptr;
    if (sensorID >= 1 && sensorID <= 4)
        parentESP32 = findModuleByPath("^.esp32_1");
    else if (sensorID >= 5 && sensorID <= 8)
        parentESP32 = findModuleByPath("^.esp32_2");
    else if (sensorID >= 9 && sensorID <= 12)
        parentESP32 = findModuleByPath("^.esp32_3");

    if (!parentESP32) {
        EV_WARN << "[PROP] Parent ESP32 not found for sensor " << sensorID << endl;
        return;
    }

    // Get positions from display strings
    cDisplayString& sensorDisplay = getDisplayString();
    cDisplayString& esp32Display = parentESP32->getDisplayString();
    
    // Parse positions from display strings (format: "p=x,y")
    std::string sensorPos = sensorDisplay.getTagArg("p", 0);
    std::string esp32Pos = esp32Display.getTagArg("p", 0);
    
    // Extract x,y coordinates
    double x1, y1, x2, y2;
    sscanf(sensorPos.c_str(), "%lf,%lf", &x1, &y1);
    sscanf(esp32Pos.c_str(), "%lf,%lf", &x2, &y2);

    // Calculate distance
    double distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    
    // Manual propagation model parameters
    const double referencePathLoss = 32.44;  // Reference path loss at 1m
    const double frequency = 868e6;          // LoRa frequency (868 MHz)
    const double pathLossExponent = 3.5;     // Path loss exponent for hilly terrain
    const double shadowingMean = 8.0;        // Mean shadowing for hillside
    const double shadowingStdDev = 4.0;      // Shadowing standard deviation
    const double attenuation = 0.8;          // Attenuation factor
    
    // Calculate path loss using log-distance path loss model
    double pathLoss = referencePathLoss + 20 * std::log10(distance) + 20 * std::log10(frequency/1e9);
    
    // Add terrain effects
    pathLoss += pathLossExponent * std::log10(distance);
    
    // Add shadowing (using normal distribution)
    double shadowing = normal(shadowingMean, shadowingStdDev);
    pathLoss += shadowing;

    // Apply attenuation
    pathLoss *= attenuation;

    EV_INFO << "\n" << std::string(50, '=') << "\n"
            << "[PROP] Sensor " << sensorID << " Propagation Details\n"
            << std::string(50, '=') << "\n"
            << "  Distance:          " << std::fixed << std::setprecision(2) << distance << " m\n"
            << "  Path Loss:         " << std::setw(5) << pathLoss << " dB\n"
            << "  Shadowing:         " << std::setw(5) << shadowing << " dB\n"
            << "  Attenuation:       " << std::setw(5) << attenuation << "\n"
            << "  Path Loss Exp:     " << std::setw(5) << pathLossExponent << "\n"
            << std::string(50, '=') << endl;
}

void C193055_HHJN::handleMessage(cMessage *msg)
{
    if (msg == timerMsg) {
        handleRTS();
    }
    else if (msg == ctsTimer) {
        handleCTS();
    }
    else if (msg == syncTimer) {
        // Periodic time synchronization
        sendTimeSync();
        // Schedule next sync
        scheduleAt(simTime() + 10, syncTimer);
    }
    else if (strcmp(msg->getName(), "TimeSync") == 0) {
        handleTimeSync(msg);
    }
    else if (isMicrocontroller) {
        // For ESP32 nodes: check message type before casting
        if (C193055_LandslideMsg *receivedData = dynamic_cast<C193055_LandslideMsg *>(msg)) {
            EV_INFO << "[RECV] ESP32 received data:\n"
                    << "  Tower Group: " << receivedData->getTowerID() << "\n"
                    << "  Rainfall:    " << std::fixed << std::setprecision(2)
                    << receivedData->getRainfall() << " mm\n"
                    << "  EMA10:       " << receivedData->getEMA10() << endl;
            send(receivedData, "out");
        } else {
            // Handle other message types or forward them
            send(msg, "out");
        }
    }
    else if (isUserNode && strcmp(msg->getName(), "Alert") == 0) {
        handleAlert(msg);
    }
    else if (isServer && strcmp(msg->getName(), "Alert") == 0) {
        // Forward alert to user with all parameters
        cMessage *alertMsg = new cMessage("Alert");
        alertMsg->addPar("sensorID").setLongValue(msg->par("sensorID").longValue());
        alertMsg->addPar("rainfall").setDoubleValue(msg->par("rainfall").doubleValue());
        alertMsg->addPar("ema10").setDoubleValue(msg->par("ema10").doubleValue());
        alertMsg->addPar("datetime").setStringValue(msg->par("datetime").stringValue());
        send(alertMsg, "out");
        delete msg;
    }
    else {
        delete msg;
    }
}

void C193055_HHJN::handleRTS()
{
    sensorState = ACTIVE_RTS;
    EV_INFO << "\n" << std::string(50, '=') << "\n"
            << "[STATE] Sensor " << sensorID << " - RTS Phase\n"
            << std::string(50, '=') << endl;
    bubble("RTS Sent");

    scheduleAt(simTime() + 0.5, ctsTimer);
}

void C193055_HHJN::handleCTS()
{
    sensorState = ACTIVE_CTS;
    EV_INFO << "\n" << std::string(50, '=') << "\n"
            << "[STATE] Sensor " << sensorID << " - CTS Phase\n"
            << std::string(50, '=') << endl;
    bubble("CTS Received");

    C193055_LandslideMsg *sensorData = new C193055_LandslideMsg("SensorData");
    CSVRow data = getNextCSVRow();
    double rainfall = data.rainfall;
    double ema10 = data.ema10;
    std::string predict = "";
    sensorData->setRainfall(rainfall);
    sensorData->setEMA10(ema10);

    int towerID = 0;
    if (sensorID >= 1 && sensorID <= 4)
        towerID = 1;
    else if (sensorID >= 5 && sensorID <= 8)
        towerID = 2;
    else if (sensorID >= 9 && sensorID <= 12)
        towerID = 3;
    sensorData->setTowerID(towerID);

    EV_INFO << "[DATA] Sensor " << std::setw(2) << sensorID
            << " | Rainfall: " << std::fixed << std::setw(6) << std::setprecision(2) << rainfall
            << " mm | EMA10: " << std::setw(6) << ema10
            << " | Tower: " << towerID << endl;

    if (rainfall > 25.4 && ema10 > 14.0) {
        EV_WARN << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
                << "!! [ALERT] Landslide potential detected at sensor " << sensorID << "\n"
                << "!!         Rainfall: " << rainfall << " mm\n"
                << "!!         EMA10:    " << ema10 << "\n"
                << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
        
        predict = "High";
        
        cMessage *alertMsg = new cMessage("Alert");
        alertMsg->addPar("sensorID").setLongValue(sensorID);
        alertMsg->addPar("rainfall").setDoubleValue(rainfall);
        alertMsg->addPar("ema10").setDoubleValue(ema10);
        alertMsg->addPar("datetime").setStringValue(data.datetime.c_str());
        send(alertMsg, "out");
    }

    else if (rainfall > 25.4 || ema10 > 14.0) {
            EV_WARN << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
                    << "!! [ALERT] Landslide potential detected at sensor " << sensorID << "\n"
                    << "!!         Rainfall: " << rainfall << " mm\n"
                    << "!!         EMA10:    " << ema10 << "\n"
                    << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;

            predict = "Medium";

            cMessage *alertMsg = new cMessage("Alert");
            alertMsg->addPar("sensorID").setLongValue(sensorID);
            alertMsg->addPar("rainfall").setDoubleValue(rainfall);
            alertMsg->addPar("ema10").setDoubleValue(ema10);
            alertMsg->addPar("datetime").setStringValue(data.datetime.c_str());
            send(alertMsg, "out");
        }

    else{
        predict = "Safe";
    }

    logAlert(sensorID, data.datetime, rainfall, ema10, predict);

    logPropagationDetails();
    send(sensorData, "out");
    sensorState = IDLE;
    EV_INFO << "[STATE] Sensor " << sensorID << " returned to IDLE state" << endl;
    scheduleAt(simTime() + uniform(5, 10), timerMsg);
}

void C193055_HHJN::logAlert(int sensorID, const std::string& datetime, double rainfall, double ema10, std::string predict)
{
    alertLogFile << datetime << "," << sensorID << "," 
                 << std::fixed << std::setprecision(2) << rainfall << "," 
                 << ema10 << ","
                 << predict << "\n";
    alertLogFile.flush();
}

void C193055_HHJN::sendTimeSync()
{
    cMessage *syncMsg = new cMessage("TimeSync");
    syncMsg->addPar("t1").setDoubleValue(simTime().dbl());
    syncMsg->addPar("t4").setDoubleValue(0.0); // Initialize t4 parameter
    send(syncMsg, "out");

    // Visualize time sync
    bubble("Time Sync");
    cDisplayString& ds = getDisplayString();
    ds.setTagArg("i", 1, "blue");
    ds.setTagArg("i", 2, "SYNC");
}

void C193055_HHJN::handleTimeSync(cMessage *msg)
{
    if (isMicrocontroller) {
        // ESP32 nodes relay time sync messages
        sendTimeSync();
    } else {
        // Sensor nodes perform pairwise synchronization
        simtime_t t1 = msg->par("t1").doubleValue();
        simtime_t t2 = simTime();
        simtime_t t3 = simTime();

        // Update t4 parameter in the message
        msg->par("t4").setDoubleValue(t3.dbl());

        performPairwiseSync(t1, t2, t3, t3);
    }
    delete msg;
}

void C193055_HHJN::performPairwiseSync(simtime_t t1, simtime_t t2, simtime_t t3, simtime_t t4)
{
    // Calculate propagation delay and clock offset
    simtime_t propagationDelay = ((t2 - t1) + (t4 - t3)) / 2;
    simtime_t clockOffset = ((t2 - t1) - (t4 - t3)) / 2;

    // Update local time and drift rate
    timeOffset = clockOffset;
    localTime = simTime() + timeOffset;

    // Calculate new drift rate based on time difference since last sync
    if (lastSyncTime > 0) {
        simtime_t timeSinceLastSync = simTime() - lastSyncTime;
        driftRate = (timeOffset - lastSyncTime) / timeSinceLastSync;
        driftRate = std::max(-MAX_DRIFT, std::min(MAX_DRIFT, driftRate));
    }

    lastSyncTime = simTime();
    isTimeSynced = true;

    EV_INFO << "\n" << std::string(50, '=') << "\n"
            << "[TIME] Node " << getName() << " Synchronization Status\n"
            << std::string(50, '=') << "\n"
            << "  Propagation Delay: " << std::fixed << std::setprecision(6) << propagationDelay << "s\n"
            << "  Clock Offset:      " << clockOffset << "s\n"
            << "  Drift Rate:        " << driftRate << "s/s\n"
            << "  Local Time:        " << localTime << "\n"
            << std::string(50, '=') << endl;
}

void C193055_HHJN::updateLocalTime(simtime_t referenceTime)
{
    simtime_t currentTime = simTime();
    timeOffset = referenceTime - currentTime;
    localTime = currentTime + timeOffset;
    isTimeSynced = true;

    EV_INFO << "[TIME] Node " << getName() 
            << " synchronized. Offset: " << timeOffset 
            << " Local time: " << localTime << endl;
}

void C193055_HHJN::sendAlertToUser(int sensorID, double rainfall, double ema10)
{
    cMessage *alertMsg = new cMessage("Alert");
    alertMsg->addPar("sensorID").setLongValue(sensorID);
    alertMsg->addPar("rainfall").setDoubleValue(rainfall);
    alertMsg->addPar("ema10").setDoubleValue(ema10);
    send(alertMsg, "out");
}

void C193055_HHJN::handleAlert(cMessage *msg)
{
    int sensorID = msg->par("sensorID").longValue();
    double rainfall = msg->par("rainfall").doubleValue();
    double ema10 = msg->par("ema10").doubleValue();
    std::string datetime = msg->par("datetime").stringValue();

    EV_INFO << "\n" << std::string(50, '!') << "\n"
            << "!! [ALERT] Landslide Warning Received!\n"
            << "!! Time:    " << datetime << "\n"
            << "!! Sensor:  " << sensorID << "\n"
            << "!! Rainfall: " << std::fixed << std::setprecision(2) << rainfall << " mm\n"
            << "!! EMA10:   " << ema10 << "\n"
            << std::string(50, '!') << endl;

    bubble("ALERT!");
    cDisplayString& ds = getDisplayString();
    ds.setTagArg("i", 1, "red");
    ds.setTagArg("i", 2, "ALERT");

    scheduleAt(simTime() + 5, new cMessage("ResetDisplay"));
    delete msg;
}

void C193055_HHJN::finish()
{
    cancelAndDelete(timerMsg);
    cancelAndDelete(ctsTimer);
    cancelAndDelete(syncTimer);
    EV_INFO << "[FINISH] Node " << getName() << " shut down" << endl;
    if (alertLogInitialized) {
        alertLogFile.close();
    }
}
