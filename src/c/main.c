#include <stdio.h>
#include "CNNModelAVX.h"

int main(int argc, char** argv) {
    //Wczytanie ParamsFile
    size_t sizeNet = sizeof(CNNModelAVXParams);
    CNNModelAVXParams* params = malloc(sizeNet);
    FILE* from = fopen("../../resources/ParamsFile4", "rb");
    fread(params, sizeNet, 1, from);
    fclose(from);

//    for(int i=0; i<64; i++)
//	    printf("%f ", params->denseoutputWeights[346000 + i]);

    //Utworzenie siatki
    CNNModelAVXNet* net;
    char* errNet = CNNModelAVXNetCreate(&net, params, 4);
    free(params);
    if (errNet) {
        printf("%s\n", errNet);
        free(errNet);
        exit(1); 
    }

    //Utworzenie silnika
    CNNModelAVXEngine* engine;
    char* errEngine = CNNModelAVXEngineCreate(&engine, net, 4);
    if (errEngine) {
        printf("%s\n", errEngine); 
        free(errEngine); 
        CNNModelAVXNetDestroy(net);
        exit(1); 
    }
    //Zarzadzanie watkami
    //     ... Use the POSIX threads API to adjust engine's threads ...

    //Wczytanie obrazu i wynik
    float* inputdataData = malloc(sizeof(float)*1*28*28);
    float* outputlogitsData = malloc(sizeof(float)*10*1*1);
    size_t sizeData = sizeof(inputdataData);

    FILE* image = fopen("../../resources/image0", "rb");
    if (image==NULL) {
        printf("Failed to open image file.\n");
        exit(1);
    }

    fread(inputdataData, sizeData, 392, image);
    fclose(image);

    CNNModelAVXEngineInference(
        engine,
        inputdataData,
        outputlogitsData
    );
    
    printf("\n");
    for (int i = 0; i < 10; i++) {
        printf("%f\t", outputlogitsData[i]);
    }

    //Zwolnienie pamieci
    free(inputdataData);
    free(outputlogitsData);
    CNNModelAVXEngineDestroy(engine);
    CNNModelAVXNetDestroy(net);
}
