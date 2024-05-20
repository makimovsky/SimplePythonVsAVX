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

    clock_t start, end;
    double t;
    FILE* ctime = fopen("../../resources/ctime.txt", "w");
    start = clock();
    char filename[35];
    for(int i=0; i<1000; i++) {
	sprintf(filename, "../../resources/images/image%d", i);
	
        FILE* image = fopen(filename, "rb");
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
	    end = clock();
            t = ((double)(end - start)) / CLOCKS_PER_SEC;
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
