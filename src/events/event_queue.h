#ifndef EVENT_QUEUE_HPP
#define EVENT_QUEUE_HPP

#include <functional>

#include <queue>
#include <vector>
#include <fstream>
#include <sstream>
#include "event.h"
#include "event_receiver.h"

using namespace std;

class EventQueue {
    
    private: 
        priority_queue<Event> events;
        unsigned long nextEventId = 0;
        std::vector<std::function<void(Event&)>> callbacks;
        double step;
             
    public:
        EventQueue() {;};    
        // Constructor to initialize the EventQueue with a CSV file path
        EventQueue(const string& path, double step = 0.001) {
            this->step = step;
            string line;
            std::ifstream csvFile;
            csvFile.open(path);
            if (!csvFile.is_open()) {
                perror("Could not open .csv file");
                exit(1);
            }           
            std::getline(csvFile, line); //skip header
            auto nextToken = [](std::stringstream& ss) -> std::string {
                std::string token;
                std::getline(ss, token, ',');
                return token;
            };    
            while (std::getline(csvFile, line)) {
                std::stringstream ss(line);

                std::string pktStr  = nextToken(ss);
                std::string recvStr = nextToken(ss);   
                std::string sendStr = nextToken(ss);
                           
                if (sendStr.empty() || pktStr.empty() || recvStr.empty())
                    continue;            
                double sendTime = std::stod(sendStr);
                unsigned long pktNr = std::stoul(pktStr);
                double rcsvTime = std::stod(recvStr);            
                schedule(pktNr, sendTime, Event::Type::SEND, 
                    [this, pktNr, sendTime](Event& event) {
                        ;//printf("SEND at %f for pkt %lu, event %lu\n", sendTime, pktNr, event.eventId);
                    });            
                schedule(pktNr, rcsvTime, Event::Type::RECEIVE, 
                    [this, pktNr, rcsvTime](Event& event) {
                        ;//printf("RECEIVE at %f for pkt %lu, event %lu\n", rcsvTime, pktNr, event.eventId);
                    });
            }
            if (csvFile.is_open()) {
                   csvFile.close();
            }
        };
        void run(double untilTime);
        bool empty() const;   
        double nextTime() const;
        void addReceiver(std::function<void(const Event&)> cb);
        //void stop();
        ~EventQueue() {;}

    private:
        void schedule(unsigned long pktNr, double time, 
                        Event::Type type, std::function<void(Event&)> action);
        void scheduleAt(double startTime, double step, double untilTime);
        void notifyReceivers(Event& event);
    };
#endif // EVENT_QUEUE_HPP
