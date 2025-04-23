#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>

#define MAX_INPUT 256
#define NUMBER_OF_ESPS 2
#define ANCHOR_DISTANCE 1

struct ServerInfo {
	char hostname[128];
	char ipAddr[INET_ADDRSTRLEN];
	char port[5];
	int serverSocket;
};

struct ESP {
	char ipAddr[INET_ADDRSTRLEN];
	char port[5];
	int espSocket;
    double distanceToTag;
};


void IP(char ipBuff[]){
	printf("IP:%s\n", ipBuff);
}
void PORT(char port[]){
	printf("PORT:%s\n", port);
}

void setServerInfo(struct ServerInfo *server){
	memset(server->hostname, '\0', 128);
	memset(server->ipAddr, '\0', INET_ADDRSTRLEN);
	struct addrinfo hints;
	struct addrinfo *res;
	struct sockaddr_in myInfo;
	socklen_t myInfoLen = sizeof(myInfo);

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM; //UDP Socket
	hints.ai_flags = AI_PASSIVE;
	int status = getaddrinfo("8.8.8.8", "1234", &hints, &res);

	int sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
	bind(sockfd, res->ai_addr, res->ai_addrlen);
	connect(sockfd, res->ai_addr, res->ai_addrlen);
	getsockname(sockfd, (struct sockaddr*)&myInfo, &myInfoLen);
	
	inet_ntop(AF_INET, &(myInfo.sin_addr), server->ipAddr, INET_ADDRSTRLEN); //set server IP
	getnameinfo((struct sockaddr*)&myInfo, myInfoLen, server->hostname, 128, NULL, 0, NI_NAMEREQD); //set server name
}

void addESP(struct ESP *esps[], int sockfd){
    // find available space in ESP array
    int i = 0;
    while(esps[i] != NULL){
        i++;
    }
    struct sockaddr_storage *incoming;
    socklen_t addrSize;
    struct sockaddr_in incomingInfo;
    socklen_t incomingInfoLen = sizeof(incomingInfo);

    esps[i] = malloc(sizeof(struct ESP));
    memset(esps[i]->ipAddr, '\0', INET_ADDRSTRLEN);
    memset(esps[i]->port, '\0', 5);
    int newfd = accept(sockfd, (struct sockaddr *)&incoming, &addrSize);
    // get ESP name, IP, and Port
    getpeername(newfd, (struct sockaddr *)&incomingInfo, &incomingInfoLen);
	inet_ntop(AF_INET, &(incomingInfo.sin_addr), esps[i]->ipAddr, INET_ADDRSTRLEN);
    snprintf(esps[i]->ipAddr, 5, "%d", ntohs(incomingInfo.sin_port));
    esps[i]->espSocket = newfd;
}

void removeAllESP(struct ESP *esps[], int nEsps){
    for(int i = 0; i < nEsps; i++){
        if(esps[i] != NULL){
            close(esps[i]->espSocket);
            free(esps[i]);
            esps[i] = NULL;
        }
    }
}


int executeCmdServer(char *cmd, char args[], struct ServerInfo *server){
	if (strncmp(cmd, "IP", 2) == 0){
		IP(server->ipAddr);
	} else if (strncmp(cmd, "PORT", 4) == 0){
		PORT(server->port);
	}
    
}

// needs change if more than 2 anchors occur
int triangulate(struct ESP *esps[]){
    double distanceBuffer[2];
    for(int i = 0; i < 2; i++){
        if(esps[i] == NULL){
            // cannot perform triangulation.  Missing anchors.
            return -1;
        }
        distanceBuffer[i] = esps[i]->distanceToTag;
    }
    // solve angles, consider anchor B's distance as side a, & vice-versa
    double a = distanceBuffer[1];
    double b = distanceBuffer[0];
    double c = ANCHOR_DISTANCE;
    float angleA = acos((b*b + c*c - a*a)/(2*b*c)) * (180.0/M_PI);
    float angleB = acos((a*a + c*c - b*b)/(2*a*c)) * (180.0/M_PI);
    printf("-------------------------------\n");
    printf("Anchor %s to Tag: (%.2f, %.2f)\n", esps[0]->ipAddr, b, angleA); //The first anchor connected to network
    printf("Anchor %s to Tag: (%.2f, %.2f)\n", esps[1]->ipAddr, a, angleB); //The second anchor connected to network
    printf("-------------------------------\n\n");
    return 0;

}

//argv[1] = port to host on
int main(int argc, char **argv){
	/*Start Here*/
    struct ServerInfo *server = malloc(sizeof(struct ServerInfo));
    setServerInfo(server);
    //create socket
    struct addrinfo hints;
    struct addrinfo *res;
    socklen_t addrSize;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM; //TCP Socket
    hints.ai_flags = AI_PASSIVE;
    memset(server->port, '\0', 6);
    strncpy(server->port, argv[1], 5);
    int status = getaddrinfo(server->ipAddr, server->port, &hints, &res);
    int sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    status = bind(sockfd, res->ai_addr, res->ai_addrlen);
    status = listen(sockfd, 10);
    server->serverSocket = sockfd;
    int socketNumber = sockfd;
    
    struct ESP *esps[NUMBER_OF_ESPS] = {NULL};

    //begin looking for input
    int exitStatus = 1;
    char input[MAX_INPUT];
    while(exitStatus){
        fd_set readfds;
        FD_ZERO(&readfds);
        //add stdin
        FD_SET(0, &readfds);
        //add input from all ESPs
        for (int i = 0; i < NUMBER_OF_ESPS; i++){
            if (esps[i] != NULL){
                FD_SET(esps[i]->espSocket, &readfds);
                if (esps[i]->espSocket > socketNumber){
                    socketNumber = esps[i]->espSocket;
                }
            }
        }
        //add listen socket
        FD_SET(server->serverSocket, &readfds);
        if(server->serverSocket > socketNumber){
            socketNumber = server->serverSocket;
        }
        

        //select inputs
        int selectResult = select(socketNumber+1, &readfds, NULL, NULL, NULL);
        //check for set stdin
        if(FD_ISSET(0, &readfds)){
            memset(input, '\0', MAX_INPUT);
            fgets(input, MAX_INPUT, stdin);
            char *cmd = strtok(input, " ");
            char *args = cmd + strlen(cmd) + 1; // +1 for '\0'
            if(strncmp(cmd, "EXIT", 4) == 0){
                removeAllESP(esps, NUMBER_OF_ESPS);
                exitStatus = 0;
            }
            executeCmdServer(cmd, args, server);
        }        
        //ESP message
        for(int i = 0; i < NUMBER_OF_ESPS; i++){
            if(esps[i] != NULL){
                double receiveDouble;
                FD_ISSET(esps[i]->espSocket, &readfds);
                //Get distance
                memset(input, '\0', MAX_INPUT);
                recv(esps[i]->espSocket, input, sizeof(double), 0);
                memcpy(&receiveDouble, input, sizeof(double));
                esps[i]->distanceToTag = receiveDouble;

                //close socket if esp is done, else print message
                //anchor doesn't have CLOSE cmd yet though
                if(strncmp(input, "CLOSE", 5) == 0){
                    printf("Closing %s\n", esps[i]->ipAddr);
                    close(esps[i]->espSocket);
                    free(esps[i]);
                    esps[i] = NULL;
                }
            }
        }
        //incoming ESP to add
        if(FD_ISSET(server->serverSocket, &readfds)){
            printf("Incoming ESP\n");
            addESP(esps, server->serverSocket);
        }
        triangulate(esps);
    }
    return 0;
	
}