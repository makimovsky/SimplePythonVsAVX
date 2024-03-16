#pragma once

// NN-512 (https://NN-512.com)
//
// Copyright (C) 2019 [
//     37ef ced3 3727 60b4
//     3c29 f9c6 dc30 d518
//     f4f3 4106 6964 cab4
//     a06f c1a3 83fd 090e
// ]
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <pthread.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" { /**/
#endif

// All weights, biases, and other trained parameters are passed into
// the initialization code through the Params struct that is declared
// just below this comment. The corresponding struct definition can be
// found near the end of this header file.
//
// Each field of the Params struct is an array of float that holds a
// parameter tensor in NCHW format with no padding. The struct fields
// are ordered by name, lexically bytewise. If you concatenate all the
// trained parameter tensors to a file in this same format and order
// you can load the struct as follows (error checking omitted here):
//
//     size_t size = sizeof(Example28Params);
//     Example28Params* to = malloc(size);
//     FILE* from = fopen("ParamsFile", "r");
//     fread(to, size, 1, from);
//     fclose(from);
//
// Be careful to match endianness (and floating point format).

typedef struct Example28Params Example28Params;

// The Net contains weights, biases, and other trained parameters in a
// form that enables efficient inference. It is created from the input
// parameter struct without modifying that struct. The input parameter
// struct is no longer needed once the Net has been created. Threads
// that are used to create the Net are temporary (in particular, those
// threads are not used for inference).
//
//     Example28Params* params = malloc(sizeof(Example28Params));
//
//     ... Load params (read from a file, perhaps) ...
//
//     Example28Net* net; // For example, 4 threads:
//     char* err = Example28NetCreate(&net, params, 4);
//     free(params);
//
//     if (err) { // Nonzero err indicates failure; net is unmodified.
//         printf("%s\n", err); // Explain the failure, add a newline.
//         free(err); // Free the error string to avoid a memory leak.
//         exit(1); // Exit, or propagate the failure some other way.
//     }
//
//     ... Perform all inference that depends on net ...
//
//     Example28NetDestroy(net);
//
// The Net can be shared and reused without restriction because it is
// never modified (not even temporarily) after being created. The Net
// should be destroyed (to free memory) once all dependent inference
// is complete.

typedef struct Example28Net Example28Net;

char* Example28NetCreate(
	Example28Net**,
	Example28Params*,
	ptrdiff_t threads
);

void Example28NetDestroy(Example28Net*);

// An Engine performs inference. It contains inference threads, scratch
// memory, and a pointer to the Net. Any number of Engines can share the
// same Net (and perform inference in parallel) because the Net is never
// modified. For best performance the number of inference threads should
// not exceed the number of CPU cores.
//
//     Example28Net* net;
//
//     ... Create net ...
//
//     Example28Engine* engine; // For example, 4 inference threads:
//     char* err = Example28EngineCreate(&engine, net, 4);
//
//     if (err) { // Nonzero err means failure; engine is unmodified.
//         printf("%s\n", err); // Explain the failure, add a newline.
//         free(err); // Free the error string to avoid a memory leak.
//
//         ... Destroy net ...
//
//         exit(1); // Exit, or propagate the failure some other way.
//     }
//
//     ... Use the POSIX threads API to adjust engine's threads ...
//     ... Use engine to perform inference (dependent on net) ...
//
//     Example28EngineDestroy(engine); // Terminate threads, free memory.
//
//     ... Destroy net ...
//
// The POSIX threads API can be used to adjust an Engine's threads. If
// an Engine has N threads, those threads are indexed 0, 1, 2, ..., N-1
// and a pthread_t identifier is associated with each index. To set the
// CPU affinity mask for the first inference thread, for example:
//
//     pthread_t thread; // The first thread has index 0:
//     char* err = Example28EnginePthreadT(engine, 0, &thread);
//
//     assert(!err); // Can only fail if the thread index is invalid.
//
//     pthread_setaffinity_np(thread, ...); // Details omitted.
//
// The inference function reads floats from (one or more) input tensors
// and writes floats to (one or more) output tensors. All the input and
// output tensors are owned (allocated and freed) by the caller and are
// in CHW format, 32-bit floating point, fully packed (in other words,
// C has the largest pitch, W has the smallest pitch, and there is no
// padding anywhere).
//
//     float* bn4Data = malloc(sizeof(float)*68*11*30);
//     float* in1Data = malloc(sizeof(float)*42*13*32);
//     float* in2Data = malloc(sizeof(float)*42*13*32);
//     float* in3Data = malloc(sizeof(float)*68*11*30);
//
//     for (...) { // Reuse the input and output tensors.
//
//         ... Write the input floats ...
//
//         Example28EngineInference( // This function cannot fail.
//             engine, // Pass an Engine as the first argument.
//             bn4Data, // The tensor arguments are sorted by name.
//             in1Data,
//             in2Data,
//             in3Data
//         );
//
//         ... Read the output floats ...
//
//     }
//
//     free(bn4Data);
//     free(in1Data);
//     free(in2Data);
//     free(in3Data);
//
// The tensor parameters of the inference function are ordered by name,
// lexically bytewise. In other words, the function parameters have been
// sorted by name using Go's "<" string comparison operator (a bytewise
// lexical string sort).

typedef struct Example28Engine Example28Engine;

char* Example28EngineCreate(
	Example28Engine**,
	Example28Net*,
	ptrdiff_t threads
);

char* Example28EnginePthreadT(
	Example28Engine*,
	ptrdiff_t threadIdx,
	pthread_t* to
);

void Example28EngineInference(
	Example28Engine*,
	float* bn4Data,
	float* in1Data,
	float* in2Data,
	float* in3Data
);

void Example28EngineDestroy(Example28Engine*);

// The fields of the following struct have been sorted by name using
// Go's "<" string comparison operator (bytewise lexical string sort).
// Tensor dimensions are NxCxHxW where N is the outermost/slowest and
// W is the innermost/fastest. There is no padding anywhere.

struct Example28Params {
	float bn1Means[42];       // 1x42x1x1
	float bn1Scales[42];      // 1x42x1x1
	float bn1Shifts[42];      // 1x42x1x1
	float bn1Variances[42];   // 1x42x1x1
	float bn2Means[42];       // 1x42x1x1
	float bn2Scales[42];      // 1x42x1x1
	float bn2Shifts[42];      // 1x42x1x1
	float bn2Variances[42];   // 1x42x1x1
	float bn3Means[68];       // 1x68x1x1
	float bn3Scales[68];      // 1x68x1x1
	float bn3Shifts[68];      // 1x68x1x1
	float bn3Variances[68];   // 1x68x1x1
	float bn4Means[68];       // 1x68x1x1
	float bn4Scales[68];      // 1x68x1x1
	float bn4Shifts[68];      // 1x68x1x1
	float bn4Variances[68];   // 1x68x1x1
	float convBiases[68];     // 1x68x1x1
	float convWeights[25704]; // 68x42x3x3
} __attribute__((packed));

#ifdef __cplusplus
/**/ }
#endif

// End of file.
