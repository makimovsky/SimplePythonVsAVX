#include "CNNModelAVX.h"

int main(int argc, char** argv) {
    //Wczytanie ParamsFile
    size_t size = sizeof(CNNModelAVXParams);
    CNNModelAVXParams* params = malloc(size);
    FILE* from = fopen("../../resources/ParamsFile", "r");
    fread(params, size, 1, from);
    fclose(from);

    //Utworzenie siatki
    CNNModelAVXNet* net;
    char* err = CNNModelAVXNetCreate(&net, params, 4);
    free(params);
    if (err) {
        printf("%s\n", err);
        free(err);
        exit(1); 
    }

    //Utworzenie silnika
    CNNModelAVXEngine* engine;
    char* err = CNNModelAVXEngineCreate(&engine, net, 4);
    if (err) {
        printf("%s\n", err); 
        free(err); 
        CNNModelAVXNetDestroy(net);
        exit(1); 
    }
    //Zarzadzanie watkami
    //     ... Use the POSIX threads API to adjust engine's threads ...

    //Wczytanie obrazu i wynik
    float* inputdataData = malloc(sizeof(float)*1*28*28);
    float* outputlogitsData = malloc(sizeof(float)*10*1*1);
    size_t size = sizeof(inputdataData);

    FILE* image = fopen("../../resources/image0", "r")
    fread(inputdataData, size, 1, image)

    CNNModelAVXEngineInference(
        engine,
        inputdataData,
        outputlogitsData
    );

    for (int i = 0; i < 10; i++) {
        printf("%f\t", outputlogitsData[i]);
    }

    //Zwolnienie pamieci
    free(inputdataData);
    free(outputlogitsData);
    CNNModelAVXEngineDestroy(engine);
    CNNModelAVXNetDestroy(net);
}
