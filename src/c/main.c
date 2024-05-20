#include <stdio.h>
#include <time.h>
#include "CNNModelAVX.h"

int main(int argc, char** argv) {
    //Wczytanie ParamsFile
    size_t sizeNet = sizeof(CNNModelAVXParams);
    CNNModelAVXParams* params = malloc(sizeNet);
    FILE* from = fopen("../../resources/ParamsFile4", "rb");
    fread(params, sizeNet, 1, from);
    fclose(from);
	
    //FILE* f = fopen("wagi.txt", "w");
    //for(int i=0; i<346112; i++)
    //	fprintf(f,"%f\n", params->denseoutputWeights[i]);
    //
    //fclose(f);
    
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

    clock_t start, t;
    FILE* ctime = fopen("../../resources/ctime.txt", "w");
    start = clock();
    for(int i=0; i<1000; i++) {
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

        if(i % 9 == 0) {
            t = ((double)(clock() - start)) / CLOCKS_PER_SEC;
            fprintf(ctime, "%f\n", t);
        }
    }
    fclose(ctime);

    // printf("\n");
    // for (int i = 0; i < 10; i++) {
    //     printf("%f\t", outputlogitsData[i]);
    // }

    //Zwolnienie pamieci
    free(inputdataData);
    free(outputlogitsData);
    CNNModelAVXEngineDestroy(engine);
    CNNModelAVXNetDestroy(net);
}
