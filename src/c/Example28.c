// To build an object file:
// gcc -c -w -std=c99 -pthread -Ofast -mavx512f Example28.c

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

#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <immintrin.h>

#include "Example28.h"

static char* Example28Errmsg1(ptrdiff_t lineNum1, char* format1, ...) {
	char* msg1 = malloc(277);
	int step1 = sprintf(msg1, "Example28: line %td: ", lineNum1);
	va_list ap1;
	va_start(ap1, format1);
	vsnprintf(msg1+step1, 277-step1, format1, ap1);
	va_end(ap1);
	return msg1;
}

typedef struct Example28ThreaderTask1 Example28ThreaderTask1;
typedef void (*Example28ThreaderCallee1)(Example28ThreaderTask1*, int64_t*);
typedef struct Example28ThreaderHub1 Example28ThreaderHub1;
typedef struct Example28ThreaderNode1 Example28ThreaderNode1;
typedef struct Example28ThreaderUnwind1 Example28ThreaderUnwind1;
typedef struct Example28ThreaderTeam1 Example28ThreaderTeam1;

struct Example28ThreaderTask1 {
	Example28ThreaderCallee1 callee1;
	void* any1;
	ptrdiff_t nd1;
	int64_t hull1[4];
};

struct Example28ThreaderHub1 {
	pthread_mutex_t mut1;
	pthread_cond_t cond1;
	ptrdiff_t pending1;
	ptrdiff_t offset1;
	long mask1;
	long status1[];
};

struct Example28ThreaderNode1 {
	pthread_mutex_t mut2;
	int64_t np1;
	int64_t pt1[4];
	Example28ThreaderTask1* task1;
	pthread_cond_t cond2;
	Example28ThreaderTeam1* team1;
	pthread_t thr1;
} __attribute__((aligned(64)));

struct Example28ThreaderUnwind1 {
	ptrdiff_t join1;
	ptrdiff_t nodeConds1;
	ptrdiff_t nodeMuts1;
	ptrdiff_t hubCond1;
	ptrdiff_t hubMut1;
	void* nodes1;
	void* hub1;
};

struct Example28ThreaderTeam1 {
	ptrdiff_t nt1;
	Example28ThreaderHub1* hub2;
	Example28ThreaderNode1* nodes2;
	Example28ThreaderUnwind1 unwind1;
};

static void Example28ThreaderInc1(
	ptrdiff_t nd2,
	int64_t*restrict hull2,
	int64_t*restrict pt2
) {
	for (ptrdiff_t i1 = 0; i1 < nd2; ++i1) {
		int64_t elem1 = pt2[i1];
		if (++elem1 == hull2[i1]) {
			pt2[i1] = 0;
		} else {
			pt2[i1] = elem1;
			break;
		}
	}
}

static void Example28ThreaderPut1(
	ptrdiff_t nd3,
	int64_t*restrict hull3,
	int64_t*restrict pt3,
	int64_t val1
) {
	ptrdiff_t i2 = 0;
	for (; i2 < nd3 && val1; ) {
		int64_t wrap1 = hull3[i2];
		int64_t carry1 = val1/wrap1;
		pt3[i2++] = val1-carry1*wrap1;
		val1 = carry1;
	}
	for (; i2 < nd3; pt3[i2++] = 0);
}

static void Example28ThreaderAdd1(
	ptrdiff_t nd4,
	int64_t*restrict hull4,
	int64_t*restrict pt4,
	int64_t*restrict plus1,
	int64_t carry2
) {
	for (ptrdiff_t i3 = 0; i3 < nd4; ++i3) {
		int64_t wrap2 = hull4[i3];
		int64_t sum1 = pt4[i3]+plus1[i3]+carry2;
		if (sum1 < wrap2) {
			pt4[i3] = sum1;
			carry2 = 0;
		} else {
			pt4[i3] = sum1-wrap2;
			carry2 = 1;
		}
	}
}

static void* Example28ThreaderMain1(void* arg1) {
	Example28ThreaderNode1* node1 = arg1;
	Example28ThreaderTeam1* team2 = node1->team1;
	ptrdiff_t nt2 = team2->nt1;
	Example28ThreaderHub1* hub3 = team2->hub2;
	Example28ThreaderNode1* nodes3 = team2->nodes2;
	size_t role1 = node1-nodes3;
	for (; __builtin_expect(pthread_mutex_lock(&node1->mut2), 0); );
	for (; ; ) {
		Example28ThreaderTask1* task2 = node1->task1;
		if (!task2) {
			for (; __builtin_expect(pthread_cond_wait(&node1->cond2, &node1->mut2), 0); );
			continue;
		}
		int64_t np2 = node1->np1;
		if (np2 < 0) {
			for (; __builtin_expect(pthread_mutex_unlock(&node1->mut2), 0); );
			return 0;
		}
		node1->task1 = 0;
		Example28ThreaderCallee1 callee2 = task2->callee1;
		ptrdiff_t nd5 = task2->nd1;
		int64_t pt5[4];
		for (; np2; np2 = node1->np1) {
			memcpy(pt5, node1->pt1, sizeof(pt5));
			node1->np1 = np2-1;
			Example28ThreaderInc1(nd5, task2->hull1, node1->pt1);
			for (; __builtin_expect(pthread_mutex_unlock(&node1->mut2), 0); );
			callee2(task2, pt5);
			for (; __builtin_expect(pthread_mutex_lock(&node1->mut2), 0); );
		}
		for (; __builtin_expect(pthread_mutex_unlock(&node1->mut2), 0); );
		for (; __builtin_expect(pthread_mutex_lock(&hub3->mut1), 0); );
		hub3->status1[role1/(sizeof(long)*8)] &= ~((long)1<<role1%(sizeof(long)*8));
		ptrdiff_t offset2 = hub3->offset1;
		long mask2 = hub3->mask1;
		ptrdiff_t wrapped1 = 0;
		for (; ; ) {
			long hand1 = hub3->status1[offset2]&mask2;
			if (!hand1) {
				++offset2;
				mask2 = -1;
				continue;
			}
			ptrdiff_t target1 = offset2*(sizeof(long)*8)+__builtin_ctzl(hand1);
			if (target1 == nt2) {
				if (wrapped1) break;
				offset2 = 0;
				mask2 = -1;
				wrapped1 = 1;
				continue;
			}
			hand1 &= -hand1;
			hub3->offset1 = offset2;
			hub3->mask1 = mask2-hand1;
			for (; __builtin_expect(pthread_mutex_unlock(&hub3->mut1), 0); );
			Example28ThreaderNode1* node2 = nodes3+target1;
			for (; __builtin_expect(pthread_mutex_lock(&node2->mut2), 0); );
			for (np2 = node2->np1; np2; np2 = node2->np1) {
				memcpy(pt5, node2->pt1, sizeof(pt5));
				node2->np1 = np2-1;
				Example28ThreaderInc1(nd5, task2->hull1, node2->pt1);
				for (; __builtin_expect(pthread_mutex_unlock(&node2->mut2), 0); );
				callee2(task2, pt5);
				for (; __builtin_expect(pthread_mutex_lock(&node2->mut2), 0); );
			}
			for (; __builtin_expect(pthread_mutex_unlock(&node2->mut2), 0); );
			for (; __builtin_expect(pthread_mutex_lock(&hub3->mut1), 0); );
			hub3->status1[offset2] &= ~hand1;
			offset2 = hub3->offset1;
			mask2 = hub3->mask1;
			wrapped1 = 0;
		}
		ptrdiff_t pending2 = --hub3->pending1;
		for (; __builtin_expect(pthread_mutex_unlock(&hub3->mut1), 0); );
		if (!pending2) for (; __builtin_expect(pthread_cond_signal(&hub3->cond1), 0); );
		for (; __builtin_expect(pthread_mutex_lock(&node1->mut2), 0); );
	}
}

static void Example28ThreaderDestroy1(Example28ThreaderTeam1* team3) {
	if (!team3) return;
	Example28ThreaderNode1* nodes4 = team3->nodes2;
	Example28ThreaderNode1* stop1 = nodes4+team3->unwind1.join1;
	for (Example28ThreaderNode1* node3 = nodes4; node3 != stop1; ++node3) {
		for (; __builtin_expect(pthread_mutex_lock(&node3->mut2), 0); );
		node3->np1 = -1;
		node3->task1 = (Example28ThreaderTask1*)1;
		for (; __builtin_expect(pthread_mutex_unlock(&node3->mut2), 0); );
		for (; __builtin_expect(pthread_cond_signal(&node3->cond2), 0); );
	}
	for (Example28ThreaderNode1* node3 = nodes4; node3 != stop1; ++node3) {
		for (; __builtin_expect(pthread_join(node3->thr1, 0), 0); );
	}
	stop1 = nodes4+team3->unwind1.nodeConds1;
	for (Example28ThreaderNode1* node3 = nodes4; node3 != stop1; ++node3) {
		for (; __builtin_expect(pthread_cond_destroy(&node3->cond2), 0); );
	}
	stop1 = nodes4+team3->unwind1.nodeMuts1;
	for (Example28ThreaderNode1* node3 = nodes4; node3 != stop1; ++node3) {
		for (; __builtin_expect(pthread_mutex_destroy(&node3->mut2), 0); );
	}
	Example28ThreaderHub1* hub4 = team3->hub2;
	if (team3->unwind1.hubCond1) {
		for (; __builtin_expect(pthread_cond_destroy(&hub4->cond1), 0); );
	}
	if (team3->unwind1.hubMut1) {
		for (; __builtin_expect(pthread_mutex_destroy(&hub4->mut1), 0); );
	}
	free(team3->unwind1.nodes1);
	free(team3->unwind1.hub1);
	free(team3);
}

static char* Example28ThreaderCreate1Up4(Example28ThreaderTeam1* team8, ptrdiff_t nt7) {
	Example28ThreaderNode1* nodes5 = team8->nodes2;
	for (Example28ThreaderNode1* node4 = nodes5; node4 != nodes5+nt7; ++node4) {
		int err2 = pthread_mutex_init(&node4->mut2, 0);
		if (__builtin_expect(err2, 0)) {
			char* msg2 = Example28Errmsg1(__LINE__, "errno %d", err2);
			team8->unwind1.nodeMuts1 = node4-nodes5;
			team8->unwind1.nodeConds1 = node4-nodes5;
			team8->unwind1.join1 = node4-nodes5;
			return msg2;
		}
		node4->task1 = 0;
		int err3 = pthread_cond_init(&node4->cond2, 0);
		if (__builtin_expect(err3, 0)) {
			char* msg3 = Example28Errmsg1(__LINE__, "errno %d", err3);
			team8->unwind1.nodeMuts1 = node4-nodes5+1;
			team8->unwind1.nodeConds1 = node4-nodes5;
			team8->unwind1.join1 = node4-nodes5;
			return msg3;
		}
		node4->team1 = team8;
		int err4 = pthread_create(&node4->thr1, 0, Example28ThreaderMain1, node4);
		if (__builtin_expect(err4, 0)) {
			char* msg4 = Example28Errmsg1(__LINE__, "errno %d", err4);
			team8->unwind1.nodeMuts1 = node4-nodes5+1;
			team8->unwind1.nodeConds1 = node4-nodes5+1;
			team8->unwind1.join1 = node4-nodes5;
			return msg4;
		}
	}
	team8->unwind1.nodeMuts1 = nt7;
	team8->unwind1.nodeConds1 = nt7;
	team8->unwind1.join1 = nt7;
	return 0;
}

static char* Example28ThreaderCreate1Up3(Example28ThreaderTeam1* team7, ptrdiff_t nt6) {
	Example28ThreaderHub1* hub5 = team7->hub2;
	int err5 = pthread_mutex_init(&hub5->mut1, 0);
	if (__builtin_expect(err5, 0)) {
		return Example28Errmsg1(__LINE__, "errno %d", err5);
	}
	team7->unwind1.hubMut1 = 1;
	int err6 = pthread_cond_init(&hub5->cond1, 0);
	if (__builtin_expect(err6, 0)) {
		return Example28Errmsg1(__LINE__, "errno %d", err6);
	}
	team7->unwind1.hubCond1 = 1;
	return Example28ThreaderCreate1Up4(team7, nt6);
}

static char* Example28ThreaderCreate1Up2(Example28ThreaderTeam1* team6, ptrdiff_t nt5) {
	size_t size2 = nt5*sizeof(Example28ThreaderNode1);
	if (__builtin_expect(size2/sizeof(Example28ThreaderNode1) != (size_t)nt5, 0)) {
		return Example28Errmsg1(__LINE__, "too many threads");
	}
	void* addr3 = malloc(size2+63);
	if (__builtin_expect(!addr3, 0)) {
		return Example28Errmsg1(__LINE__, "errno %d", errno);
	}
	team6->unwind1.nodes1 = addr3;
	team6->nodes2 = (void*)(((size_t)addr3+63)&-64);
	return Example28ThreaderCreate1Up3(team6, nt5);
}

static char* Example28ThreaderCreate1Up1(Example28ThreaderTeam1* team5, ptrdiff_t nt4) {
	team5->nt1 = nt4;
	size_t size1 = sizeof(Example28ThreaderHub1);
	size1 += sizeof(long)*((size_t)nt4/(sizeof(long)*8)+1);
	size1 = (size1+63)&-64;
	void* addr2 = malloc(size1+63);
	if (__builtin_expect(!addr2, 0)) {
		return Example28Errmsg1(__LINE__, "errno %d", errno);
	}
	team5->unwind1.hub1 = addr2;
	team5->hub2 = (void*)(((size_t)addr2+63)&-64);
	return Example28ThreaderCreate1Up2(team5, nt4);
}

static char* Example28ThreaderCreate1(Example28ThreaderTeam1** team4, ptrdiff_t nt3) {
	if (__builtin_expect(nt3 < 1, 0)) {
		return Example28Errmsg1(__LINE__, "too few threads");
	}
	void* addr1 = calloc(1, sizeof(Example28ThreaderTeam1));
	if (__builtin_expect(!addr1, 0)) {
		return Example28Errmsg1(__LINE__, "errno %d", errno);
	}
	char* err1 = Example28ThreaderCreate1Up1(addr1, nt3);
	if (__builtin_expect(!!err1, 0)) {
		Example28ThreaderDestroy1(addr1);
	} else {
		*team4 = addr1;
	}
	return err1;
}

static char* Example28ThreaderPthreadT1(
	pthread_t* thr2,
	Example28ThreaderTeam1* team9,
	ptrdiff_t idx1
) {
	if (__builtin_expect(idx1 < 0 || idx1 >= team9->nt1, 0)) {
		return Example28Errmsg1(__LINE__, "bad thread idx");
	}
	*thr2 = team9->nodes2[idx1].thr1;
	return 0;
}

static void Example28ThreaderDo1(Example28ThreaderTeam1* team10, Example28ThreaderTask1* task3) {
	ptrdiff_t nd6 = task3->nd1;
	if (nd6 < 1) return;
	int64_t tot1 = task3->hull1[0];
	for (ptrdiff_t i4 = 1; i4 < nd6; tot1 *= task3->hull1[i4++]);
	ptrdiff_t nt8 = team10->nt1;
	int64_t each1 = tot1/nt8;
	ptrdiff_t more1 = tot1%nt8;
	int64_t plus2[4];
	Example28ThreaderPut1(nd6, task3->hull1, plus2, each1);
	int64_t pt6[4] = {0};
	Example28ThreaderHub1* hub6 = team10->hub2;
	for (; __builtin_expect(pthread_mutex_lock(&hub6->mut1), 0); );
	Example28ThreaderNode1* node5 = team10->nodes2;
	for (ptrdiff_t i4 = 0; ; ++node5) {
		for (; __builtin_expect(pthread_mutex_lock(&node5->mut2), 0); );
		int64_t carry3 = i4 < more1;
		node5->np1 = each1+carry3;
		memcpy(node5->pt1, pt6, sizeof(pt6));
		node5->task1 = task3;
		for (; __builtin_expect(pthread_mutex_unlock(&node5->mut2), 0); );
		for (; __builtin_expect(pthread_cond_signal(&node5->cond2), 0); );
		if (++i4 == nt8) break;
		Example28ThreaderAdd1(nd6, task3->hull1, pt6, plus2, carry3);
	}
	hub6->offset1 = 0;
	hub6->mask1 = -1;
	for (ptrdiff_t i4 = (size_t)nt8/(sizeof(long)*8); i4 >= 0; ) {
		hub6->status1[i4--] = -1;
	}
	for (hub6->pending1 = nt8; hub6->pending1; ) {
		for (; __builtin_expect(pthread_cond_wait(&hub6->cond1, &hub6->mut1), 0); );
	}
	for (; __builtin_expect(pthread_mutex_unlock(&hub6->mut1), 0); );
}

static __m512 Example28Exp1(__m512 x1) {
	x1 = _mm512_max_ps(x1, _mm512_set1_ps(-8.733654e+01f));
	x1 = _mm512_min_ps(x1, _mm512_set1_ps(8.872284e+01f));
	__m512 t1 = _mm512_mul_ps(x1, _mm512_set1_ps(1.442695e+00f));
	__m512 r1 = _mm512_roundscale_ps(t1, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC);
	__m512 f1 = _mm512_fmadd_ps(r1, _mm512_set1_ps(-6.9314575e-01f), x1);
	f1 = _mm512_fmadd_ps(r1, _mm512_set1_ps(-1.4286068e-06f), f1);
	__m512 g1 = _mm512_set1_ps(4.194439e-02f);
	g1 = _mm512_fmadd_ps(g1, f1, _mm512_set1_ps(1.6800667e-01f));
	g1 = _mm512_fmadd_ps(g1, f1, _mm512_set1_ps(4.9999994e-01f));
	g1 = _mm512_fmadd_ps(g1, f1, _mm512_set1_ps(9.999569e-01f));
	g1 = _mm512_fmadd_ps(g1, f1, _mm512_set1_ps(9.9999964e-01f));
	__m512i y1 = _mm512_slli_epi32(_mm512_cvtps_epi32(t1), 23);
	return _mm512_castsi512_ps(_mm512_add_epi32(y1, _mm512_castps_si512(g1)));
}

static __m512 Example28Rsqrt1(__m512 x2) {
	__m512 y2 = _mm512_rsqrt14_ps(x2);
	__m512 z1 = _mm512_mul_ps(x2, y2);
	__m512 a1 = _mm512_mul_ps(y2, _mm512_set1_ps(5e-01f));
	__m512 b1 = _mm512_fnmadd_ps(y2, z1, _mm512_set1_ps(3e+00f));
	return _mm512_mul_ps(a1, b1);
}

static void Example28BnSimplify1(
	float*restrict means1,
	float*restrict variances1,
	float*restrict scales1,
	float*restrict shifts1,
	char*restrict mas1
) {
	__m512 eps1 = _mm512_set1_ps(1e-05f);
	__m512i xlo1 = _mm512_set_epi32(23, 7, 22, 6, 21, 5, 20, 4, 19, 3, 18, 2, 17, 1, 16, 0);
	__m512i xhi1 = _mm512_set_epi32(31, 15, 30, 14, 29, 13, 28, 12, 27, 11, 26, 10, 25, 9, 24, 8);
	__m512 va1 = _mm512_loadu_ps(variances1+(ptrdiff_t)16*0);
	__m512 va2 = _mm512_loadu_ps(variances1+(ptrdiff_t)16*1);
	__m512 va3 = _mm512_maskz_loadu_ps(1023, variances1+(ptrdiff_t)16*2);
	__m512 rcp1 = Example28Rsqrt1(_mm512_add_ps(eps1, va1));
	__m512 rcp2 = Example28Rsqrt1(_mm512_add_ps(eps1, va2));
	__m512 rcp3 = Example28Rsqrt1(_mm512_add_ps(eps1, va3));
	__m512 sc1 = _mm512_loadu_ps(scales1+(ptrdiff_t)16*0);
	__m512 sc2 = _mm512_loadu_ps(scales1+(ptrdiff_t)16*1);
	__m512 sc3 = _mm512_maskz_loadu_ps(1023, scales1+(ptrdiff_t)16*2);
	__m512 mul1 = _mm512_mul_ps(rcp1, sc1);
	__m512 mul2 = _mm512_mul_ps(rcp2, sc2);
	__m512 mul3 = _mm512_mul_ps(rcp3, sc3);
	__m512 me1 = _mm512_loadu_ps(means1+(ptrdiff_t)16*0);
	__m512 me2 = _mm512_loadu_ps(means1+(ptrdiff_t)16*1);
	__m512 me3 = _mm512_maskz_loadu_ps(1023, means1+(ptrdiff_t)16*2);
	__m512 sh1 = _mm512_loadu_ps(shifts1+(ptrdiff_t)16*0);
	__m512 sh2 = _mm512_loadu_ps(shifts1+(ptrdiff_t)16*1);
	__m512 sh3 = _mm512_maskz_loadu_ps(1023, shifts1+(ptrdiff_t)16*2);
	__m512 add1 = _mm512_fnmadd_ps(me1, mul1, sh1);
	__m512 add2 = _mm512_fnmadd_ps(me2, mul2, sh2);
	__m512 add3 = _mm512_fnmadd_ps(me3, mul3, sh3);
	__m512 lo1 = _mm512_permutex2var_ps(mul1, xlo1, add1);
	__m512 lo2 = _mm512_permutex2var_ps(mul2, xlo1, add2);
	__m512 lo3 = _mm512_permutex2var_ps(mul3, xlo1, add3);
	__m512 hi1 = _mm512_permutex2var_ps(mul1, xhi1, add1);
	__m512 hi2 = _mm512_permutex2var_ps(mul2, xhi1, add2);
	__m512 hi3 = _mm512_permutex2var_ps(mul3, xhi1, add3);
	_mm512_storeu_ps(mas1+(ptrdiff_t)64*0, lo1);
	_mm512_storeu_ps(mas1+(ptrdiff_t)64*1, hi1);
	_mm512_storeu_ps(mas1+(ptrdiff_t)64*2, lo2);
	_mm512_storeu_ps(mas1+(ptrdiff_t)64*3, hi2);
	_mm512_storeu_ps(mas1+(ptrdiff_t)64*4, lo3);
	_mm512_mask_storeu_ps(mas1+(ptrdiff_t)64*5, 15, hi3);
}

static void Example28BnSimplify2(
	float*restrict means2,
	float*restrict variances2,
	float*restrict scales2,
	float*restrict shifts2,
	char*restrict mas2
) {
	__m512 eps2 = _mm512_set1_ps(1e-05f);
	__m512i xlo2 = _mm512_set_epi32(23, 7, 22, 6, 21, 5, 20, 4, 19, 3, 18, 2, 17, 1, 16, 0);
	__m512i xhi2 = _mm512_set_epi32(31, 15, 30, 14, 29, 13, 28, 12, 27, 11, 26, 10, 25, 9, 24, 8);
	__m512 va4 = _mm512_loadu_ps(variances2+(ptrdiff_t)16*0);
	__m512 va5 = _mm512_loadu_ps(variances2+(ptrdiff_t)16*1);
	__m512 va6 = _mm512_loadu_ps(variances2+(ptrdiff_t)16*2);
	__m512 va7 = _mm512_loadu_ps(variances2+(ptrdiff_t)16*3);
	__m512 va8 = _mm512_maskz_loadu_ps(15, variances2+(ptrdiff_t)16*4);
	__m512 rcp4 = Example28Rsqrt1(_mm512_add_ps(eps2, va4));
	__m512 rcp5 = Example28Rsqrt1(_mm512_add_ps(eps2, va5));
	__m512 rcp6 = Example28Rsqrt1(_mm512_add_ps(eps2, va6));
	__m512 rcp7 = Example28Rsqrt1(_mm512_add_ps(eps2, va7));
	__m512 rcp8 = Example28Rsqrt1(_mm512_add_ps(eps2, va8));
	__m512 sc4 = _mm512_loadu_ps(scales2+(ptrdiff_t)16*0);
	__m512 sc5 = _mm512_loadu_ps(scales2+(ptrdiff_t)16*1);
	__m512 sc6 = _mm512_loadu_ps(scales2+(ptrdiff_t)16*2);
	__m512 sc7 = _mm512_loadu_ps(scales2+(ptrdiff_t)16*3);
	__m512 sc8 = _mm512_maskz_loadu_ps(15, scales2+(ptrdiff_t)16*4);
	__m512 mul4 = _mm512_mul_ps(rcp4, sc4);
	__m512 mul5 = _mm512_mul_ps(rcp5, sc5);
	__m512 mul6 = _mm512_mul_ps(rcp6, sc6);
	__m512 mul7 = _mm512_mul_ps(rcp7, sc7);
	__m512 mul8 = _mm512_mul_ps(rcp8, sc8);
	__m512 me4 = _mm512_loadu_ps(means2+(ptrdiff_t)16*0);
	__m512 me5 = _mm512_loadu_ps(means2+(ptrdiff_t)16*1);
	__m512 me6 = _mm512_loadu_ps(means2+(ptrdiff_t)16*2);
	__m512 me7 = _mm512_loadu_ps(means2+(ptrdiff_t)16*3);
	__m512 me8 = _mm512_maskz_loadu_ps(15, means2+(ptrdiff_t)16*4);
	__m512 sh4 = _mm512_loadu_ps(shifts2+(ptrdiff_t)16*0);
	__m512 sh5 = _mm512_loadu_ps(shifts2+(ptrdiff_t)16*1);
	__m512 sh6 = _mm512_loadu_ps(shifts2+(ptrdiff_t)16*2);
	__m512 sh7 = _mm512_loadu_ps(shifts2+(ptrdiff_t)16*3);
	__m512 sh8 = _mm512_maskz_loadu_ps(15, shifts2+(ptrdiff_t)16*4);
	__m512 add4 = _mm512_fnmadd_ps(me4, mul4, sh4);
	__m512 add5 = _mm512_fnmadd_ps(me5, mul5, sh5);
	__m512 add6 = _mm512_fnmadd_ps(me6, mul6, sh6);
	__m512 add7 = _mm512_fnmadd_ps(me7, mul7, sh7);
	__m512 add8 = _mm512_fnmadd_ps(me8, mul8, sh8);
	__m512 lo4 = _mm512_permutex2var_ps(mul4, xlo2, add4);
	__m512 lo5 = _mm512_permutex2var_ps(mul5, xlo2, add5);
	__m512 lo6 = _mm512_permutex2var_ps(mul6, xlo2, add6);
	__m512 lo7 = _mm512_permutex2var_ps(mul7, xlo2, add7);
	__m512 lo8 = _mm512_permutex2var_ps(mul8, xlo2, add8);
	__m512 hi4 = _mm512_permutex2var_ps(mul4, xhi2, add4);
	__m512 hi5 = _mm512_permutex2var_ps(mul5, xhi2, add5);
	__m512 hi6 = _mm512_permutex2var_ps(mul6, xhi2, add6);
	__m512 hi7 = _mm512_permutex2var_ps(mul7, xhi2, add7);
	_mm512_storeu_ps(mas2+(ptrdiff_t)64*0, lo4);
	_mm512_storeu_ps(mas2+(ptrdiff_t)64*1, hi4);
	_mm512_storeu_ps(mas2+(ptrdiff_t)64*2, lo5);
	_mm512_storeu_ps(mas2+(ptrdiff_t)64*3, hi5);
	_mm512_storeu_ps(mas2+(ptrdiff_t)64*4, lo6);
	_mm512_storeu_ps(mas2+(ptrdiff_t)64*5, hi6);
	_mm512_storeu_ps(mas2+(ptrdiff_t)64*6, lo7);
	_mm512_storeu_ps(mas2+(ptrdiff_t)64*7, hi7);
	_mm512_mask_storeu_ps(mas2+(ptrdiff_t)64*8, 255, lo8);
}

static void Example28ThreeArrangeFilts1Callee1(Example28ThreaderTask1* task4, int64_t* pt7) {
	char** tensors2 = task4->any1;
	ptrdiff_t b2 = pt7[0];
	ptrdiff_t g2 = 0;
	ptrdiff_t e1 = 0;
	char*restrict bfPtr1 = tensors2[4]+272*e1;
	char*restrict wfPtr1 = tensors2[4]+320+3446784*e1;
	char*restrict wtPtr1 = tensors2[0]+14256*e1;
	char*restrict biasPtr1 = tensors2[1];
	char*restrict bnPtr1 = tensors2[2]+(ptrdiff_t)8*396*e1;
	char*restrict bnPtr2 = tensors2[3];
	ptrdiff_t i5 = 1*g2;
	ptrdiff_t j1 = 4*b2;
	ptrdiff_t jj1 = j1+(b2 < 3 ? 3 : 4);
	if (j1 < 17) {
		for (; j1 != 17; ++j1) {
			ptrdiff_t k1 = 0+1*j1;
			ptrdiff_t cut1 = 0;
			__m512 bf1 = _mm512_setzero_ps();
			__m512 bf2 = _mm512_setzero_ps();
			__m512 bf3 = _mm512_setzero_ps();
			__m512 bf4 = _mm512_setzero_ps();
			__m512 postMul1 = _mm512_set1_ps(((float*)bnPtr2+(ptrdiff_t)2*(0+68*i5+4*j1))[0]);
			__m512 postMul2 = _mm512_set1_ps(((float*)bnPtr2+(ptrdiff_t)2*(1+68*i5+4*j1))[0]);
			__m512 postMul3 = _mm512_set1_ps(((float*)bnPtr2+(ptrdiff_t)2*(2+68*i5+4*j1))[0]);
			__m512 postMul4 = _mm512_set1_ps(((float*)bnPtr2+(ptrdiff_t)2*(3+68*i5+4*j1))[0]);
			ptrdiff_t s1 = 0;
			for (; s1 != 42; ++s1) {
				__m512 wt1 = _mm512_maskz_loadu_ps(511, wtPtr1+0+102816*i5+6048*j1+36*s1);
				__m512 wt2 = _mm512_maskz_loadu_ps(511, wtPtr1+1512+102816*i5+6048*j1+36*s1);
				__m512 wt3 = _mm512_maskz_loadu_ps(511, wtPtr1+3024+102816*i5+6048*j1+36*s1);
				__m512 wt4 = _mm512_maskz_loadu_ps(511, wtPtr1+4536+102816*i5+6048*j1+36*s1);
				wt1 = _mm512_mul_ps(wt1, postMul1);
				wt2 = _mm512_mul_ps(wt2, postMul2);
				wt3 = _mm512_mul_ps(wt3, postMul3);
				wt4 = _mm512_mul_ps(wt4, postMul4);
				__m512 preMul1 = _mm512_set1_ps(((float*)bnPtr1+(ptrdiff_t)2*(0+42*i5+1*s1))[0]);
				__m512 preAdd1 = _mm512_set1_ps(((float*)bnPtr1+(ptrdiff_t)2*(0+42*i5+1*s1))[1]);
				bf1 = _mm512_fmadd_ps(wt1, preAdd1, bf1);
				bf2 = _mm512_fmadd_ps(wt2, preAdd1, bf2);
				bf3 = _mm512_fmadd_ps(wt3, preAdd1, bf3);
				bf4 = _mm512_fmadd_ps(wt4, preAdd1, bf4);
				wt1 = _mm512_mul_ps(wt1, preMul1);
				wt2 = _mm512_mul_ps(wt2, preMul1);
				wt3 = _mm512_mul_ps(wt3, preMul1);
				wt4 = _mm512_mul_ps(wt4, preMul1);
				__m512i pm1 = _mm512_set_epi32(22, 8, 7, 6, 21, 20, 19, 5, 4, 3, 18, 17, 16, 2, 1, 0);
				__m512i pm2 = _mm512_set_epi32(28, 14, 13, 12, 27, 26, 25, 11, 10, 9, 24, 23, 22, 8, 7, 6);
				__m512 tmp1 = _mm512_permutex2var_ps(wt1, pm1, wt3);
				__m512 tmp2 = _mm512_permutex2var_ps(wt2, pm1, wt4);
				__m512 tmp3 = _mm512_permutex2var_ps(wt1, pm2, wt3);
				__m512 tmp4 = _mm512_permutex2var_ps(wt2, pm2, wt4);
				__m512 in1 = _mm512_permutex2var_ps(tmp1, pm1, tmp2);
				__m512 in2 = _mm512_permutex2var_ps(tmp1, pm2, tmp2);
				__m512 in3 = _mm512_permutex2var_ps(tmp3, pm1, tmp4);
				__m512 tmp5 = _mm512_fmadd_ps(in1, _mm512_set1_ps(4e+00f), in3);
				__m512 tmp6 = _mm512_add_ps(in1, in3);
				__m512 tmp7 = _mm512_fmadd_ps(in3, _mm512_set1_ps(4e+00f), in1);
				__m512 tmp8 = _mm512_add_ps(in2, tmp6);
				__m512 tmp9 = _mm512_fmadd_ps(in2, _mm512_set1_ps(2e+00f), tmp7);
				tmp7 = _mm512_fnmadd_ps(in2, _mm512_set1_ps(2e+00f), tmp7);
				__m512 tmp10 = _mm512_fnmadd_ps(in2, _mm512_set1_ps(2e+00f), tmp5);
				tmp5 = _mm512_fmadd_ps(in2, _mm512_set1_ps(2e+00f), tmp5);
				tmp6 = _mm512_sub_ps(tmp6, in2);
				__m512 tmp27 = _mm512_unpacklo_ps(in1, tmp8);
				__m512 tmp28 = _mm512_unpackhi_ps(in1, tmp8);
				__m512 tmp29 = _mm512_unpacklo_ps(tmp6, tmp9);
				__m512 tmp30 = _mm512_unpackhi_ps(tmp6, tmp9);
				__m512 tmp31 = _mm512_unpacklo_ps(tmp7, tmp5);
				__m512 tmp32 = _mm512_unpackhi_ps(tmp7, tmp5);
				__m512 tmp33 = _mm512_unpacklo_ps(tmp10, in3);
				__m512 tmp34 = _mm512_unpackhi_ps(tmp10, in3);
				__m512 tmp35 = _mm512_shuffle_ps(tmp27, tmp29, 68);
				__m512 tmp36 = _mm512_shuffle_ps(tmp27, tmp29, 238);
				__m512 tmp37 = _mm512_shuffle_ps(tmp28, tmp30, 68);
				__m512 tmp38 = _mm512_shuffle_ps(tmp28, tmp30, 238);
				__m512 tmp39 = _mm512_shuffle_ps(tmp31, tmp33, 68);
				__m512 tmp40 = _mm512_shuffle_ps(tmp31, tmp33, 238);
				__m512 tmp41 = _mm512_shuffle_ps(tmp32, tmp34, 68);
				__m512 tmp42 = _mm512_shuffle_ps(tmp32, tmp34, 238);
				__m512 tmp43 = _mm512_shuffle_f32x4(tmp35, tmp39, 136);
				__m512 tmp44 = _mm512_shuffle_f32x4(tmp35, tmp39, 221);
				__m512 tmp45 = _mm512_shuffle_f32x4(tmp36, tmp40, 136);
				__m512 tmp46 = _mm512_shuffle_f32x4(tmp36, tmp40, 221);
				__m512 tmp47 = _mm512_shuffle_f32x4(tmp37, tmp41, 136);
				__m512 tmp48 = _mm512_shuffle_f32x4(tmp37, tmp41, 221);
				__m512 tmp49 = _mm512_shuffle_f32x4(tmp38, tmp42, 136);
				__m512 tmp50 = _mm512_shuffle_f32x4(tmp38, tmp42, 221);
				in1 = _mm512_shuffle_f32x4(tmp43, tmp43, 136);
				__m512 tmp11 = _mm512_shuffle_f32x4(tmp43, tmp43, 221);
				tmp8 = _mm512_shuffle_f32x4(tmp45, tmp45, 136);
				__m512 tmp12 = _mm512_shuffle_f32x4(tmp45, tmp45, 221);
				tmp6 = _mm512_shuffle_f32x4(tmp47, tmp47, 136);
				__m512 tmp13 = _mm512_shuffle_f32x4(tmp47, tmp47, 221);
				tmp9 = _mm512_shuffle_f32x4(tmp49, tmp49, 136);
				__m512 tmp14 = _mm512_shuffle_f32x4(tmp49, tmp49, 221);
				tmp7 = _mm512_shuffle_f32x4(tmp44, tmp44, 136);
				tmp5 = _mm512_shuffle_f32x4(tmp46, tmp46, 136);
				tmp10 = _mm512_shuffle_f32x4(tmp48, tmp48, 136);
				in3 = _mm512_shuffle_f32x4(tmp50, tmp50, 136);
				in1 = _mm512_shuffle_f32x4(in1, tmp9, 68);
				tmp8 = _mm512_shuffle_f32x4(tmp8, tmp7, 68);
				tmp6 = _mm512_shuffle_f32x4(tmp6, tmp5, 68);
				tmp10 = _mm512_shuffle_f32x4(tmp10, tmp12, 68);
				in3 = _mm512_shuffle_f32x4(in3, tmp13, 68);
				tmp11 = _mm512_shuffle_f32x4(tmp11, tmp14, 68);
				__m512 tmp15 = _mm512_fmadd_ps(in1, _mm512_set1_ps(4e+00f), tmp6);
				__m512 tmp21 = _mm512_fmadd_ps(tmp10, _mm512_set1_ps(4e+00f), tmp11);
				__m512 tmp16 = _mm512_add_ps(in1, tmp6);
				__m512 tmp22 = _mm512_add_ps(tmp10, tmp11);
				__m512 tmp17 = _mm512_fmadd_ps(tmp6, _mm512_set1_ps(4e+00f), in1);
				__m512 tmp23 = _mm512_fmadd_ps(tmp11, _mm512_set1_ps(4e+00f), tmp10);
				__m512 tmp18 = _mm512_add_ps(tmp8, tmp16);
				__m512 tmp24 = _mm512_add_ps(in3, tmp22);
				__m512 tmp19 = _mm512_fmadd_ps(tmp8, _mm512_set1_ps(2e+00f), tmp17);
				__m512 tmp25 = _mm512_fmadd_ps(in3, _mm512_set1_ps(2e+00f), tmp23);
				tmp17 = _mm512_fnmadd_ps(tmp8, _mm512_set1_ps(2e+00f), tmp17);
				tmp23 = _mm512_fnmadd_ps(in3, _mm512_set1_ps(2e+00f), tmp23);
				__m512 tmp20 = _mm512_fnmadd_ps(tmp8, _mm512_set1_ps(2e+00f), tmp15);
				__m512 tmp26 = _mm512_fnmadd_ps(in3, _mm512_set1_ps(2e+00f), tmp21);
				tmp15 = _mm512_fmadd_ps(tmp8, _mm512_set1_ps(2e+00f), tmp15);
				tmp21 = _mm512_fmadd_ps(in3, _mm512_set1_ps(2e+00f), tmp21);
				tmp16 = _mm512_sub_ps(tmp16, tmp8);
				tmp22 = _mm512_sub_ps(tmp22, in3);
				in1 = _mm512_mul_ps(in1, _mm512_set_ps(1e+00f, 5.5555557e-03f, 5.5555557e-03f, 1.1111111e-02f, 1.1111111e-02f, -2.2222222e-01f, -2.2222222e-01f, 1e+00f, 1e+00f, 5.5555557e-03f, 5.5555557e-03f, 1.1111111e-02f, 1.1111111e-02f, -2.2222222e-01f, -2.2222222e-01f, 1e+00f));
				tmp18 = _mm512_mul_ps(tmp18, _mm512_set_ps(-2.2222222e-01f, -1.234568e-03f, -1.234568e-03f, -2.469136e-03f, -2.469136e-03f, 4.9382716e-02f, 4.9382716e-02f, -2.2222222e-01f, -2.2222222e-01f, -1.234568e-03f, -1.234568e-03f, -2.469136e-03f, -2.469136e-03f, 4.9382716e-02f, 4.9382716e-02f, -2.2222222e-01f));
				tmp16 = _mm512_mul_ps(tmp16, _mm512_set_ps(-2.2222222e-01f, -1.234568e-03f, -1.234568e-03f, -2.469136e-03f, -2.469136e-03f, 4.9382716e-02f, 4.9382716e-02f, -2.2222222e-01f, -2.2222222e-01f, -1.234568e-03f, -1.234568e-03f, -2.469136e-03f, -2.469136e-03f, 4.9382716e-02f, 4.9382716e-02f, -2.2222222e-01f));
				tmp19 = _mm512_mul_ps(tmp19, _mm512_set_ps(1.1111111e-02f, 6.1728395e-05f, 6.1728395e-05f, 1.2345679e-04f, 1.2345679e-04f, -2.469136e-03f, -2.469136e-03f, 1.1111111e-02f, 1.1111111e-02f, 6.1728395e-05f, 6.1728395e-05f, 1.2345679e-04f, 1.2345679e-04f, -2.469136e-03f, -2.469136e-03f, 1.1111111e-02f));
				tmp17 = _mm512_mul_ps(tmp17, _mm512_set_ps(1.1111111e-02f, 6.1728395e-05f, 6.1728395e-05f, 1.2345679e-04f, 1.2345679e-04f, -2.469136e-03f, -2.469136e-03f, 1.1111111e-02f, 1.1111111e-02f, 6.1728395e-05f, 6.1728395e-05f, 1.2345679e-04f, 1.2345679e-04f, -2.469136e-03f, -2.469136e-03f, 1.1111111e-02f));
				tmp15 = _mm512_mul_ps(tmp15, _mm512_set_ps(5.5555557e-03f, 3.0864197e-05f, 3.0864197e-05f, 6.1728395e-05f, 6.1728395e-05f, -1.234568e-03f, -1.234568e-03f, 5.5555557e-03f, 5.5555557e-03f, 3.0864197e-05f, 3.0864197e-05f, 6.1728395e-05f, 6.1728395e-05f, -1.234568e-03f, -1.234568e-03f, 5.5555557e-03f));
				tmp20 = _mm512_mul_ps(tmp20, _mm512_set_ps(5.5555557e-03f, 3.0864197e-05f, 3.0864197e-05f, 6.1728395e-05f, 6.1728395e-05f, -1.234568e-03f, -1.234568e-03f, 5.5555557e-03f, 5.5555557e-03f, 3.0864197e-05f, 3.0864197e-05f, 6.1728395e-05f, 6.1728395e-05f, -1.234568e-03f, -1.234568e-03f, 5.5555557e-03f));
				tmp6 = _mm512_mul_ps(tmp6, _mm512_set_ps(1e+00f, 5.5555557e-03f, 5.5555557e-03f, 1.1111111e-02f, 1.1111111e-02f, -2.2222222e-01f, -2.2222222e-01f, 1e+00f, 1e+00f, 5.5555557e-03f, 5.5555557e-03f, 1.1111111e-02f, 1.1111111e-02f, -2.2222222e-01f, -2.2222222e-01f, 1e+00f));
				tmp10 = _mm512_mul_ps(tmp10, _mm512_set_ps(1e+00f, 5.5555557e-03f, 5.5555557e-03f, 1.1111111e-02f, 1.1111111e-02f, -2.2222222e-01f, -2.2222222e-01f, 1e+00f, 1e+00f, 5.5555557e-03f, 5.5555557e-03f, 1.1111111e-02f, 1.1111111e-02f, -2.2222222e-01f, -2.2222222e-01f, 1e+00f));
				tmp24 = _mm512_mul_ps(tmp24, _mm512_set_ps(-2.2222222e-01f, -1.234568e-03f, -1.234568e-03f, -2.469136e-03f, -2.469136e-03f, 4.9382716e-02f, 4.9382716e-02f, -2.2222222e-01f, -2.2222222e-01f, -1.234568e-03f, -1.234568e-03f, -2.469136e-03f, -2.469136e-03f, 4.9382716e-02f, 4.9382716e-02f, -2.2222222e-01f));
				tmp22 = _mm512_mul_ps(tmp22, _mm512_set_ps(-2.2222222e-01f, -1.234568e-03f, -1.234568e-03f, -2.469136e-03f, -2.469136e-03f, 4.9382716e-02f, 4.9382716e-02f, -2.2222222e-01f, -2.2222222e-01f, -1.234568e-03f, -1.234568e-03f, -2.469136e-03f, -2.469136e-03f, 4.9382716e-02f, 4.9382716e-02f, -2.2222222e-01f));
				tmp25 = _mm512_mul_ps(tmp25, _mm512_set_ps(1.1111111e-02f, 6.1728395e-05f, 6.1728395e-05f, 1.2345679e-04f, 1.2345679e-04f, -2.469136e-03f, -2.469136e-03f, 1.1111111e-02f, 1.1111111e-02f, 6.1728395e-05f, 6.1728395e-05f, 1.2345679e-04f, 1.2345679e-04f, -2.469136e-03f, -2.469136e-03f, 1.1111111e-02f));
				tmp23 = _mm512_mul_ps(tmp23, _mm512_set_ps(1.1111111e-02f, 6.1728395e-05f, 6.1728395e-05f, 1.2345679e-04f, 1.2345679e-04f, -2.469136e-03f, -2.469136e-03f, 1.1111111e-02f, 1.1111111e-02f, 6.1728395e-05f, 6.1728395e-05f, 1.2345679e-04f, 1.2345679e-04f, -2.469136e-03f, -2.469136e-03f, 1.1111111e-02f));
				tmp21 = _mm512_mul_ps(tmp21, _mm512_set_ps(5.5555557e-03f, 3.0864197e-05f, 3.0864197e-05f, 6.1728395e-05f, 6.1728395e-05f, -1.234568e-03f, -1.234568e-03f, 5.5555557e-03f, 5.5555557e-03f, 3.0864197e-05f, 3.0864197e-05f, 6.1728395e-05f, 6.1728395e-05f, -1.234568e-03f, -1.234568e-03f, 5.5555557e-03f));
				tmp26 = _mm512_mul_ps(tmp26, _mm512_set_ps(5.5555557e-03f, 3.0864197e-05f, 3.0864197e-05f, 6.1728395e-05f, 6.1728395e-05f, -1.234568e-03f, -1.234568e-03f, 5.5555557e-03f, 5.5555557e-03f, 3.0864197e-05f, 3.0864197e-05f, 6.1728395e-05f, 6.1728395e-05f, -1.234568e-03f, -1.234568e-03f, 5.5555557e-03f));
				tmp11 = _mm512_mul_ps(tmp11, _mm512_set_ps(1e+00f, 5.5555557e-03f, 5.5555557e-03f, 1.1111111e-02f, 1.1111111e-02f, -2.2222222e-01f, -2.2222222e-01f, 1e+00f, 1e+00f, 5.5555557e-03f, 5.5555557e-03f, 1.1111111e-02f, 1.1111111e-02f, -2.2222222e-01f, -2.2222222e-01f, 1e+00f));
				__m512 out1 = _mm512_shuffle_f32x4(in1, tmp18, 68);
				__m512 out5 = _mm512_shuffle_f32x4(in1, tmp18, 238);
				__m512 out2 = _mm512_shuffle_f32x4(tmp16, tmp19, 68);
				__m512 out6 = _mm512_shuffle_f32x4(tmp16, tmp19, 238);
				__m512 out3 = _mm512_shuffle_f32x4(tmp17, tmp15, 68);
				__m512 out7 = _mm512_shuffle_f32x4(tmp17, tmp15, 238);
				__m512 out4 = _mm512_shuffle_f32x4(tmp20, tmp6, 68);
				__m512 out8 = _mm512_shuffle_f32x4(tmp20, tmp6, 238);
				__m512 out9 = _mm512_shuffle_f32x4(tmp10, tmp24, 68);
				__m512 out13 = _mm512_shuffle_f32x4(tmp10, tmp24, 238);
				__m512 out10 = _mm512_shuffle_f32x4(tmp22, tmp25, 68);
				__m512 out14 = _mm512_shuffle_f32x4(tmp22, tmp25, 238);
				__m512 out11 = _mm512_shuffle_f32x4(tmp23, tmp21, 68);
				__m512 out15 = _mm512_shuffle_f32x4(tmp23, tmp21, 238);
				__m512 out12 = _mm512_shuffle_f32x4(tmp26, tmp11, 68);
				__m512 out16 = _mm512_shuffle_f32x4(tmp26, tmp11, 238);
				ptrdiff_t off1 = 32*cut1;
				ptrdiff_t off2 = (size_t)(cut1+1)/4*5376+(size_t)(cut1+1)%4*32;
				ptrdiff_t off3 = (size_t)(cut1+2)/4*5376+(size_t)(cut1+2)%4*32;
				ptrdiff_t off4 = (size_t)(cut1+3)/4*5376+(size_t)(cut1+3)%4*32;
				__m512i wf1 = _mm512_castsi256_si512(_mm512_cvtps_ph(out1, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf2 = _mm512_castsi256_si512(_mm512_cvtps_ph(out5, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf3 = _mm512_castsi256_si512(_mm512_cvtps_ph(out9, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf4 = _mm512_castsi256_si512(_mm512_cvtps_ph(out13, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf5 = _mm512_castsi256_si512(_mm512_cvtps_ph(out2, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf6 = _mm512_castsi256_si512(_mm512_cvtps_ph(out6, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf7 = _mm512_castsi256_si512(_mm512_cvtps_ph(out10, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf8 = _mm512_castsi256_si512(_mm512_cvtps_ph(out14, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf9 = _mm512_castsi256_si512(_mm512_cvtps_ph(out3, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf10 = _mm512_castsi256_si512(_mm512_cvtps_ph(out7, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf11 = _mm512_castsi256_si512(_mm512_cvtps_ph(out11, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf12 = _mm512_castsi256_si512(_mm512_cvtps_ph(out15, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf13 = _mm512_castsi256_si512(_mm512_cvtps_ph(out4, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf14 = _mm512_castsi256_si512(_mm512_cvtps_ph(out8, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf15 = _mm512_castsi256_si512(_mm512_cvtps_ph(out12, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				__m512i wf16 = _mm512_castsi256_si512(_mm512_cvtps_ph(out16, _MM_FROUND_TO_NEAREST_INT|_MM_FROUND_NO_EXC));
				_mm512_mask_storeu_epi32(wfPtr1+0+365568*i5+5376*k1+off1+128*s1, 255, wf1);
				_mm512_mask_storeu_epi32(wfPtr1+0+365568*i5+5376*k1+off2+128*s1, 255, wf2);
				_mm512_mask_storeu_epi32(wfPtr1+0+365568*i5+5376*k1+off3+128*s1, 255, wf3);
				_mm512_mask_storeu_epi32(wfPtr1+0+365568*i5+5376*k1+off4+128*s1, 255, wf4);
				_mm512_mask_storeu_epi32(wfPtr1+91392+365568*i5+5376*k1+off1+128*s1, 255, wf5);
				_mm512_mask_storeu_epi32(wfPtr1+91392+365568*i5+5376*k1+off2+128*s1, 255, wf6);
				_mm512_mask_storeu_epi32(wfPtr1+91392+365568*i5+5376*k1+off3+128*s1, 255, wf7);
				_mm512_mask_storeu_epi32(wfPtr1+91392+365568*i5+5376*k1+off4+128*s1, 255, wf8);
				_mm512_mask_storeu_epi32(wfPtr1+182784+365568*i5+5376*k1+off1+128*s1, 255, wf9);
				_mm512_mask_storeu_epi32(wfPtr1+182784+365568*i5+5376*k1+off2+128*s1, 255, wf10);
				_mm512_mask_storeu_epi32(wfPtr1+182784+365568*i5+5376*k1+off3+128*s1, 255, wf11);
				_mm512_mask_storeu_epi32(wfPtr1+182784+365568*i5+5376*k1+off4+128*s1, 255, wf12);
				_mm512_mask_storeu_epi32(wfPtr1+274176+365568*i5+5376*k1+off1+128*s1, 255, wf13);
				_mm512_mask_storeu_epi32(wfPtr1+274176+365568*i5+5376*k1+off2+128*s1, 255, wf14);
				_mm512_mask_storeu_epi32(wfPtr1+274176+365568*i5+5376*k1+off3+128*s1, 255, wf15);
				_mm512_mask_storeu_epi32(wfPtr1+274176+365568*i5+5376*k1+off4+128*s1, 255, wf16);
			}
			__m512i pmEven1 = _mm512_set_epi32(30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0);
			__m512i pmOdd1 = _mm512_set_epi32(31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1);
			__m512i pm4Lo1 = _mm512_set_epi32(27, 26, 25, 24, 11, 10, 9, 8, 19, 18, 17, 16, 3, 2, 1, 0);
			__m512i pm4Hi1 = _mm512_set_epi32(31, 30, 29, 28, 15, 14, 13, 12, 23, 22, 21, 20, 7, 6, 5, 4);
			__m512 upper3 = _mm512_shuffle_f32x4(bf1, bf1, 14);
			__m512 upper4 = _mm512_shuffle_f32x4(bf3, bf3, 14);
			bf1 = _mm512_add_ps(bf1, upper3);
			bf3 = _mm512_add_ps(bf3, upper4);
			__m512 upper6 = _mm512_shuffle_f32x4(bf2, bf2, 14);
			__m512 upper7 = _mm512_shuffle_f32x4(bf4, bf4, 14);
			bf2 = _mm512_add_ps(bf2, upper6);
			bf4 = _mm512_add_ps(bf4, upper7);
			__m512 upper2 = _mm512_permutex2var_ps(bf1, pm4Hi1, bf3);
			__m512 upper5 = _mm512_permutex2var_ps(bf2, pm4Hi1, bf4);
			bf1 = _mm512_permutex2var_ps(bf1, pm4Lo1, bf3);
			bf2 = _mm512_permutex2var_ps(bf2, pm4Lo1, bf4);
			bf1 = _mm512_add_ps(bf1, upper2);
			bf2 = _mm512_add_ps(bf2, upper5);
			__m512 upper1 = _mm512_shuffle_ps(bf1, bf2, 238);
			bf1 = _mm512_shuffle_ps(bf1, bf2, 68);
			bf1 = _mm512_add_ps(bf1, upper1);
			__m512 upper8 = _mm512_permutexvar_ps(pmOdd1, bf1);
			bf1 = _mm512_permutexvar_ps(pmEven1, bf1);
			bf1 = _mm512_add_ps(bf1, upper8);
			__m512 bias1 = _mm512_setzero_ps();
			if (!e1) {
				bias1 = _mm512_maskz_loadu_ps(15, biasPtr1-0+272*i5+16*j1);
				__m512i pmMul1 = _mm512_set_epi32(30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8, 6, 4, 2, 0);
				__m512i pmAdd1 = _mm512_set_epi32(31, 29, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1);
				__m512 mas3 = _mm512_maskz_loadu_ps(255, bnPtr2+(ptrdiff_t)8*(0+68*i5+4*j1));
				__m512 postMul5 = _mm512_permutexvar_ps(pmMul1, mas3);
				__m512 postAdd1 = _mm512_permutexvar_ps(pmAdd1, mas3);
				bias1 = _mm512_fmadd_ps(bias1, postMul5, postAdd1);
				bf1 = _mm512_add_ps(bf1, bias1);
			}
			_mm512_mask_storeu_ps(bfPtr1-0+272*i5+16*j1, 15, bf1);
			if (j1 >= jj1) return;
		}
	}
}

static void Example28ThreeArrangeFilts1(Example28ThreaderTeam1* team13, char** tensors1) {
	Example28ThreaderTask1 task5;
	task5.callee1 = Example28ThreeArrangeFilts1Callee1;
	task5.any1 = tensors1;
	task5.nd1 = 3;
	task5.hull1[0] = 4;
	task5.hull1[1] = 1;
	task5.hull1[2] = 1;
	Example28ThreaderDo1(team13, &task5);
}

static void Example28ThreeArrangeDats1Callee1(Example28ThreaderTask1* task6, int64_t* pt8) {
	char** tensors4 = task6->any1;
	ptrdiff_t s2 = 0;
	ptrdiff_t c1 = 0;
	ptrdiff_t g3 = 0;
	ptrdiff_t e2 = 0;
	(void)pt8;
	char*restrict datPtr1 = tensors4[0]-0+658944*e2;
	char*restrict bnPtr3 = tensors4[1]+(ptrdiff_t)8*396*e2;
	char*restrict datPtr2 = tensors4[2]-0+658944*e2;
	char*restrict dfPtr1 = tensors4[3]+1013760*e2;
	ptrdiff_t i6 = 1*g3;
	ptrdiff_t j2 = 2*c1;
	ptrdiff_t rel1 = j2-0;
	ptrdiff_t base1 = 0;
	if (rel1 < 1) {
		ptrdiff_t h1 = base1+0;
		ptrdiff_t w1 = 0;
		ptrdiff_t k2 = 0;
		for (; k2 != 21; ++k2) {
			__m512 dat1 = _mm512_maskz_loadu_ps(16383, datPtr1+0+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			__m512 bnMul1 = _mm512_set1_ps(((float*)bnPtr3+(ptrdiff_t)2*(0+42*i6+42*s2+2*k2))[0]);
			__m512 bnAdd1 = _mm512_set1_ps(((float*)bnPtr3+(ptrdiff_t)2*(0+42*i6+42*s2+2*k2))[1]);
			dat1 = _mm512_mask_fmadd_ps(dat1, 16383, bnMul1, bnAdd1);
			__mmask16 mask3 = _mm512_cmp_ps_mask(dat1, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat1 = _mm512_mask_mul_ps(dat1, mask3, dat1, _mm512_set1_ps(6.25e-01f));
			dat1 = _mm512_add_ps(dat1, _mm512_maskz_loadu_ps(16383, datPtr2+0+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat2 = _mm512_maskz_loadu_ps(16383, datPtr1+48+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat2 = _mm512_mask_fmadd_ps(dat2, 16383, bnMul1, bnAdd1);
			__mmask16 mask4 = _mm512_cmp_ps_mask(dat2, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat2 = _mm512_mask_mul_ps(dat2, mask4, dat2, _mm512_set1_ps(6.25e-01f));
			dat2 = _mm512_add_ps(dat2, _mm512_maskz_loadu_ps(16383, datPtr2+48+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512i pm3 = _mm512_set_epi32(13, 12, 11, 10, 9, 8, 7, 6, 7, 6, 5, 4, 3, 2, 1, 0);
			__m512 in4 = _mm512_permutexvar_ps(pm3, dat1);
			__m512 in12 = _mm512_permutexvar_ps(pm3, dat2);
			__m512 dat3 = _mm512_maskz_loadu_ps(16383, datPtr1+128+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat3 = _mm512_mask_fmadd_ps(dat3, 16383, bnMul1, bnAdd1);
			__mmask16 mask5 = _mm512_cmp_ps_mask(dat3, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat3 = _mm512_mask_mul_ps(dat3, mask5, dat3, _mm512_set1_ps(6.25e-01f));
			dat3 = _mm512_add_ps(dat3, _mm512_maskz_loadu_ps(16383, datPtr2+128+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat4 = _mm512_maskz_loadu_ps(16383, datPtr1+176+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat4 = _mm512_mask_fmadd_ps(dat4, 16383, bnMul1, bnAdd1);
			__mmask16 mask6 = _mm512_cmp_ps_mask(dat4, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat4 = _mm512_mask_mul_ps(dat4, mask6, dat4, _mm512_set1_ps(6.25e-01f));
			dat4 = _mm512_add_ps(dat4, _mm512_maskz_loadu_ps(16383, datPtr2+176+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in5 = _mm512_permutexvar_ps(pm3, dat3);
			__m512 in13 = _mm512_permutexvar_ps(pm3, dat4);
			__m512 dat5 = _mm512_maskz_loadu_ps(16383, datPtr1+256+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat5 = _mm512_mask_fmadd_ps(dat5, 16383, bnMul1, bnAdd1);
			__mmask16 mask7 = _mm512_cmp_ps_mask(dat5, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat5 = _mm512_mask_mul_ps(dat5, mask7, dat5, _mm512_set1_ps(6.25e-01f));
			dat5 = _mm512_add_ps(dat5, _mm512_maskz_loadu_ps(16383, datPtr2+256+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat6 = _mm512_maskz_loadu_ps(16383, datPtr1+304+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat6 = _mm512_mask_fmadd_ps(dat6, 16383, bnMul1, bnAdd1);
			__mmask16 mask8 = _mm512_cmp_ps_mask(dat6, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat6 = _mm512_mask_mul_ps(dat6, mask8, dat6, _mm512_set1_ps(6.25e-01f));
			dat6 = _mm512_add_ps(dat6, _mm512_maskz_loadu_ps(16383, datPtr2+304+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in6 = _mm512_permutexvar_ps(pm3, dat5);
			__m512 in14 = _mm512_permutexvar_ps(pm3, dat6);
			__m512 dat7 = _mm512_maskz_loadu_ps(16383, datPtr1+384+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat7 = _mm512_mask_fmadd_ps(dat7, 16383, bnMul1, bnAdd1);
			__mmask16 mask9 = _mm512_cmp_ps_mask(dat7, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat7 = _mm512_mask_mul_ps(dat7, mask9, dat7, _mm512_set1_ps(6.25e-01f));
			dat7 = _mm512_add_ps(dat7, _mm512_maskz_loadu_ps(16383, datPtr2+384+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat8 = _mm512_maskz_loadu_ps(16383, datPtr1+432+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat8 = _mm512_mask_fmadd_ps(dat8, 16383, bnMul1, bnAdd1);
			__mmask16 mask10 = _mm512_cmp_ps_mask(dat8, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat8 = _mm512_mask_mul_ps(dat8, mask10, dat8, _mm512_set1_ps(6.25e-01f));
			dat8 = _mm512_add_ps(dat8, _mm512_maskz_loadu_ps(16383, datPtr2+432+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in7 = _mm512_permutexvar_ps(pm3, dat7);
			__m512 in15 = _mm512_permutexvar_ps(pm3, dat8);
			__m512 dat9 = _mm512_maskz_loadu_ps(16383, datPtr1+512+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat9 = _mm512_mask_fmadd_ps(dat9, 16383, bnMul1, bnAdd1);
			__mmask16 mask11 = _mm512_cmp_ps_mask(dat9, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat9 = _mm512_mask_mul_ps(dat9, mask11, dat9, _mm512_set1_ps(6.25e-01f));
			dat9 = _mm512_add_ps(dat9, _mm512_maskz_loadu_ps(16383, datPtr2+512+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat10 = _mm512_maskz_loadu_ps(16383, datPtr1+560+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat10 = _mm512_mask_fmadd_ps(dat10, 16383, bnMul1, bnAdd1);
			__mmask16 mask12 = _mm512_cmp_ps_mask(dat10, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat10 = _mm512_mask_mul_ps(dat10, mask12, dat10, _mm512_set1_ps(6.25e-01f));
			dat10 = _mm512_add_ps(dat10, _mm512_maskz_loadu_ps(16383, datPtr2+560+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in8 = _mm512_permutexvar_ps(pm3, dat9);
			__m512 in16 = _mm512_permutexvar_ps(pm3, dat10);
			__m512 dat11 = _mm512_maskz_loadu_ps(16383, datPtr1+640+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat11 = _mm512_mask_fmadd_ps(dat11, 16383, bnMul1, bnAdd1);
			__mmask16 mask13 = _mm512_cmp_ps_mask(dat11, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat11 = _mm512_mask_mul_ps(dat11, mask13, dat11, _mm512_set1_ps(6.25e-01f));
			dat11 = _mm512_add_ps(dat11, _mm512_maskz_loadu_ps(16383, datPtr2+640+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat12 = _mm512_maskz_loadu_ps(16383, datPtr1+688+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat12 = _mm512_mask_fmadd_ps(dat12, 16383, bnMul1, bnAdd1);
			__mmask16 mask14 = _mm512_cmp_ps_mask(dat12, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat12 = _mm512_mask_mul_ps(dat12, mask14, dat12, _mm512_set1_ps(6.25e-01f));
			dat12 = _mm512_add_ps(dat12, _mm512_maskz_loadu_ps(16383, datPtr2+688+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in9 = _mm512_permutexvar_ps(pm3, dat11);
			__m512 in17 = _mm512_permutexvar_ps(pm3, dat12);
			__m512 dat13 = _mm512_maskz_loadu_ps(16383, datPtr1+768+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat13 = _mm512_mask_fmadd_ps(dat13, 16383, bnMul1, bnAdd1);
			__mmask16 mask15 = _mm512_cmp_ps_mask(dat13, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat13 = _mm512_mask_mul_ps(dat13, mask15, dat13, _mm512_set1_ps(6.25e-01f));
			dat13 = _mm512_add_ps(dat13, _mm512_maskz_loadu_ps(16383, datPtr2+768+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat14 = _mm512_maskz_loadu_ps(16383, datPtr1+816+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat14 = _mm512_mask_fmadd_ps(dat14, 16383, bnMul1, bnAdd1);
			__mmask16 mask16 = _mm512_cmp_ps_mask(dat14, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat14 = _mm512_mask_mul_ps(dat14, mask16, dat14, _mm512_set1_ps(6.25e-01f));
			dat14 = _mm512_add_ps(dat14, _mm512_maskz_loadu_ps(16383, datPtr2+816+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in10 = _mm512_permutexvar_ps(pm3, dat13);
			__m512 in18 = _mm512_permutexvar_ps(pm3, dat14);
			__m512 dat15 = _mm512_maskz_loadu_ps(16383, datPtr1+896+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat15 = _mm512_mask_fmadd_ps(dat15, 16383, bnMul1, bnAdd1);
			__mmask16 mask17 = _mm512_cmp_ps_mask(dat15, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat15 = _mm512_mask_mul_ps(dat15, mask17, dat15, _mm512_set1_ps(6.25e-01f));
			dat15 = _mm512_add_ps(dat15, _mm512_maskz_loadu_ps(16383, datPtr2+896+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat16 = _mm512_maskz_loadu_ps(16383, datPtr1+944+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat16 = _mm512_mask_fmadd_ps(dat16, 16383, bnMul1, bnAdd1);
			__mmask16 mask18 = _mm512_cmp_ps_mask(dat16, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat16 = _mm512_mask_mul_ps(dat16, mask18, dat16, _mm512_set1_ps(6.25e-01f));
			dat16 = _mm512_add_ps(dat16, _mm512_maskz_loadu_ps(16383, datPtr2+944+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in11 = _mm512_permutexvar_ps(pm3, dat15);
			__m512 in19 = _mm512_permutexvar_ps(pm3, dat16);
			__m512 tmp51 = _mm512_add_ps(in5, in9);
			__m512 tmp55 = _mm512_add_ps(in13, in17);
			__m512 tmp52 = _mm512_sub_ps(in8, in6);
			__m512 tmp56 = _mm512_sub_ps(in16, in14);
			__m512 tmp53 = _mm512_add_ps(in6, in10);
			__m512 tmp57 = _mm512_add_ps(in14, in18);
			in4 = _mm512_sub_ps(in4, in10);
			in12 = _mm512_sub_ps(in12, in18);
			tmp51 = _mm512_fmadd_ps(in7, _mm512_set1_ps(-4.25e+00f), tmp51);
			tmp55 = _mm512_fmadd_ps(in15, _mm512_set1_ps(-4.25e+00f), tmp55);
			tmp53 = _mm512_fmadd_ps(in8, _mm512_set1_ps(-4.25e+00f), tmp53);
			tmp57 = _mm512_fmadd_ps(in16, _mm512_set1_ps(-4.25e+00f), tmp57);
			in4 = _mm512_fmadd_ps(tmp52, _mm512_set1_ps(5.25e+00f), in4);
			in12 = _mm512_fmadd_ps(tmp56, _mm512_set1_ps(5.25e+00f), in12);
			tmp52 = _mm512_fmadd_ps(in6, _mm512_set1_ps(2.5e-01f), in10);
			tmp56 = _mm512_fmadd_ps(in14, _mm512_set1_ps(2.5e-01f), in18);
			in6 = _mm512_fmadd_ps(in6, _mm512_set1_ps(4e+00f), in10);
			in14 = _mm512_fmadd_ps(in14, _mm512_set1_ps(4e+00f), in18);
			__m512 tmp54 = _mm512_sub_ps(tmp53, tmp51);
			__m512 tmp58 = _mm512_sub_ps(tmp57, tmp55);
			tmp53 = _mm512_add_ps(tmp51, tmp53);
			tmp57 = _mm512_add_ps(tmp55, tmp57);
			tmp51 = _mm512_fmadd_ps(in5, _mm512_set1_ps(2.5e-01f), in9);
			tmp55 = _mm512_fmadd_ps(in13, _mm512_set1_ps(2.5e-01f), in17);
			tmp52 = _mm512_fmadd_ps(in8, _mm512_set1_ps(-1.25e+00f), tmp52);
			tmp56 = _mm512_fmadd_ps(in16, _mm512_set1_ps(-1.25e+00f), tmp56);
			in8 = _mm512_fmadd_ps(in8, _mm512_set1_ps(-5e+00f), in6);
			in16 = _mm512_fmadd_ps(in16, _mm512_set1_ps(-5e+00f), in14);
			tmp51 = _mm512_fmadd_ps(in7, _mm512_set1_ps(-1.25e+00f), tmp51);
			tmp55 = _mm512_fmadd_ps(in15, _mm512_set1_ps(-1.25e+00f), tmp55);
			in10 = _mm512_fmadd_ps(tmp51, _mm512_set1_ps(2e+00f), tmp52);
			in18 = _mm512_fmadd_ps(tmp55, _mm512_set1_ps(2e+00f), tmp56);
			tmp52 = _mm512_fnmadd_ps(tmp51, _mm512_set1_ps(2e+00f), tmp52);
			tmp56 = _mm512_fnmadd_ps(tmp55, _mm512_set1_ps(2e+00f), tmp56);
			tmp51 = _mm512_fmadd_ps(in9, _mm512_set1_ps(2.5e-01f), in5);
			tmp55 = _mm512_fmadd_ps(in17, _mm512_set1_ps(2.5e-01f), in13);
			in5 = _mm512_sub_ps(in11, in5);
			in13 = _mm512_sub_ps(in19, in13);
			tmp51 = _mm512_fmadd_ps(in7, _mm512_set1_ps(-1.25e+00f), tmp51);
			tmp55 = _mm512_fmadd_ps(in15, _mm512_set1_ps(-1.25e+00f), tmp55);
			in7 = _mm512_sub_ps(in7, in9);
			in15 = _mm512_sub_ps(in15, in17);
			in7 = _mm512_fmadd_ps(in7, _mm512_set1_ps(5.25e+00f), in5);
			in15 = _mm512_fmadd_ps(in15, _mm512_set1_ps(5.25e+00f), in13);
			in6 = _mm512_fmadd_ps(tmp51, _mm512_set1_ps(2e+00f), in8);
			in14 = _mm512_fmadd_ps(tmp55, _mm512_set1_ps(2e+00f), in16);
			in8 = _mm512_fnmadd_ps(tmp51, _mm512_set1_ps(2e+00f), in8);
			in16 = _mm512_fnmadd_ps(tmp55, _mm512_set1_ps(2e+00f), in16);
			__m512 tmp67 = _mm512_unpacklo_ps(in4, tmp53);
			__m512 tmp68 = _mm512_unpackhi_ps(in4, tmp53);
			__m512 tmp69 = _mm512_unpacklo_ps(tmp54, in10);
			__m512 tmp70 = _mm512_unpackhi_ps(tmp54, in10);
			__m512 tmp71 = _mm512_unpacklo_ps(tmp52, in6);
			__m512 tmp72 = _mm512_unpackhi_ps(tmp52, in6);
			__m512 tmp73 = _mm512_unpacklo_ps(in8, in7);
			__m512 tmp74 = _mm512_unpackhi_ps(in8, in7);
			__m512 tmp75 = _mm512_unpacklo_ps(in12, tmp57);
			__m512 tmp76 = _mm512_unpackhi_ps(in12, tmp57);
			__m512 tmp77 = _mm512_unpacklo_ps(tmp58, in18);
			__m512 tmp78 = _mm512_unpackhi_ps(tmp58, in18);
			__m512 tmp79 = _mm512_unpacklo_ps(tmp56, in14);
			__m512 tmp80 = _mm512_unpackhi_ps(tmp56, in14);
			__m512 tmp81 = _mm512_unpacklo_ps(in16, in15);
			__m512 tmp82 = _mm512_unpackhi_ps(in16, in15);
			__m512 tmp83 = _mm512_shuffle_ps(tmp67, tmp69, 68);
			__m512 tmp84 = _mm512_shuffle_ps(tmp67, tmp69, 238);
			__m512 tmp85 = _mm512_shuffle_ps(tmp68, tmp70, 68);
			__m512 tmp86 = _mm512_shuffle_ps(tmp68, tmp70, 238);
			__m512 tmp87 = _mm512_shuffle_ps(tmp71, tmp73, 68);
			__m512 tmp88 = _mm512_shuffle_ps(tmp71, tmp73, 238);
			__m512 tmp89 = _mm512_shuffle_ps(tmp72, tmp74, 68);
			__m512 tmp90 = _mm512_shuffle_ps(tmp72, tmp74, 238);
			__m512 tmp91 = _mm512_shuffle_ps(tmp75, tmp77, 68);
			__m512 tmp92 = _mm512_shuffle_ps(tmp75, tmp77, 238);
			__m512 tmp93 = _mm512_shuffle_ps(tmp76, tmp78, 68);
			__m512 tmp94 = _mm512_shuffle_ps(tmp76, tmp78, 238);
			__m512 tmp95 = _mm512_shuffle_ps(tmp79, tmp81, 68);
			__m512 tmp96 = _mm512_shuffle_ps(tmp79, tmp81, 238);
			__m512 tmp97 = _mm512_shuffle_ps(tmp80, tmp82, 68);
			__m512 tmp98 = _mm512_shuffle_ps(tmp80, tmp82, 238);
			__m512 tmp99 = _mm512_shuffle_f32x4(tmp83, tmp87, 136);
			__m512 tmp100 = _mm512_shuffle_f32x4(tmp83, tmp87, 221);
			__m512 tmp101 = _mm512_shuffle_f32x4(tmp84, tmp88, 136);
			__m512 tmp102 = _mm512_shuffle_f32x4(tmp84, tmp88, 221);
			__m512 tmp103 = _mm512_shuffle_f32x4(tmp85, tmp89, 136);
			__m512 tmp104 = _mm512_shuffle_f32x4(tmp85, tmp89, 221);
			__m512 tmp105 = _mm512_shuffle_f32x4(tmp86, tmp90, 136);
			__m512 tmp106 = _mm512_shuffle_f32x4(tmp86, tmp90, 221);
			__m512 tmp107 = _mm512_shuffle_f32x4(tmp91, tmp95, 136);
			__m512 tmp108 = _mm512_shuffle_f32x4(tmp91, tmp95, 221);
			__m512 tmp109 = _mm512_shuffle_f32x4(tmp92, tmp96, 136);
			__m512 tmp110 = _mm512_shuffle_f32x4(tmp92, tmp96, 221);
			__m512 tmp111 = _mm512_shuffle_f32x4(tmp93, tmp97, 136);
			__m512 tmp112 = _mm512_shuffle_f32x4(tmp93, tmp97, 221);
			__m512 tmp113 = _mm512_shuffle_f32x4(tmp94, tmp98, 136);
			__m512 tmp114 = _mm512_shuffle_f32x4(tmp94, tmp98, 221);
			in4 = _mm512_shuffle_f32x4(tmp99, tmp107, 136);
			in12 = _mm512_shuffle_f32x4(tmp99, tmp107, 221);
			tmp53 = _mm512_shuffle_f32x4(tmp101, tmp109, 136);
			tmp57 = _mm512_shuffle_f32x4(tmp101, tmp109, 221);
			tmp54 = _mm512_shuffle_f32x4(tmp103, tmp111, 136);
			tmp58 = _mm512_shuffle_f32x4(tmp103, tmp111, 221);
			in10 = _mm512_shuffle_f32x4(tmp105, tmp113, 136);
			in18 = _mm512_shuffle_f32x4(tmp105, tmp113, 221);
			tmp52 = _mm512_shuffle_f32x4(tmp100, tmp108, 136);
			tmp56 = _mm512_shuffle_f32x4(tmp100, tmp108, 221);
			in6 = _mm512_shuffle_f32x4(tmp102, tmp110, 136);
			in14 = _mm512_shuffle_f32x4(tmp102, tmp110, 221);
			in8 = _mm512_shuffle_f32x4(tmp104, tmp112, 136);
			in16 = _mm512_shuffle_f32x4(tmp104, tmp112, 221);
			in7 = _mm512_shuffle_f32x4(tmp106, tmp114, 136);
			in15 = _mm512_shuffle_f32x4(tmp106, tmp114, 221);
			__m512 tmp59 = _mm512_add_ps(tmp53, in6);
			__m512 tmp63 = _mm512_add_ps(tmp57, in14);
			__m512 tmp60 = _mm512_sub_ps(tmp52, tmp54);
			__m512 tmp64 = _mm512_sub_ps(tmp56, tmp58);
			__m512 tmp61 = _mm512_add_ps(tmp54, in8);
			__m512 tmp65 = _mm512_add_ps(tmp58, in16);
			in4 = _mm512_sub_ps(in4, in8);
			in12 = _mm512_sub_ps(in12, in16);
			tmp59 = _mm512_fmadd_ps(in10, _mm512_set1_ps(-4.25e+00f), tmp59);
			tmp63 = _mm512_fmadd_ps(in18, _mm512_set1_ps(-4.25e+00f), tmp63);
			tmp61 = _mm512_fmadd_ps(tmp52, _mm512_set1_ps(-4.25e+00f), tmp61);
			tmp65 = _mm512_fmadd_ps(tmp56, _mm512_set1_ps(-4.25e+00f), tmp65);
			in4 = _mm512_fmadd_ps(tmp60, _mm512_set1_ps(5.25e+00f), in4);
			in12 = _mm512_fmadd_ps(tmp64, _mm512_set1_ps(5.25e+00f), in12);
			tmp60 = _mm512_fmadd_ps(tmp54, _mm512_set1_ps(2.5e-01f), in8);
			tmp64 = _mm512_fmadd_ps(tmp58, _mm512_set1_ps(2.5e-01f), in16);
			tmp54 = _mm512_fmadd_ps(tmp54, _mm512_set1_ps(4e+00f), in8);
			tmp58 = _mm512_fmadd_ps(tmp58, _mm512_set1_ps(4e+00f), in16);
			__m512 tmp62 = _mm512_sub_ps(tmp61, tmp59);
			__m512 tmp66 = _mm512_sub_ps(tmp65, tmp63);
			tmp61 = _mm512_add_ps(tmp59, tmp61);
			tmp65 = _mm512_add_ps(tmp63, tmp65);
			tmp59 = _mm512_fmadd_ps(tmp53, _mm512_set1_ps(2.5e-01f), in6);
			tmp63 = _mm512_fmadd_ps(tmp57, _mm512_set1_ps(2.5e-01f), in14);
			tmp60 = _mm512_fmadd_ps(tmp52, _mm512_set1_ps(-1.25e+00f), tmp60);
			tmp64 = _mm512_fmadd_ps(tmp56, _mm512_set1_ps(-1.25e+00f), tmp64);
			tmp52 = _mm512_fmadd_ps(tmp52, _mm512_set1_ps(-5e+00f), tmp54);
			tmp56 = _mm512_fmadd_ps(tmp56, _mm512_set1_ps(-5e+00f), tmp58);
			tmp59 = _mm512_fmadd_ps(in10, _mm512_set1_ps(-1.25e+00f), tmp59);
			tmp63 = _mm512_fmadd_ps(in18, _mm512_set1_ps(-1.25e+00f), tmp63);
			in8 = _mm512_fmadd_ps(tmp59, _mm512_set1_ps(2e+00f), tmp60);
			in16 = _mm512_fmadd_ps(tmp63, _mm512_set1_ps(2e+00f), tmp64);
			tmp60 = _mm512_fnmadd_ps(tmp59, _mm512_set1_ps(2e+00f), tmp60);
			tmp64 = _mm512_fnmadd_ps(tmp63, _mm512_set1_ps(2e+00f), tmp64);
			tmp59 = _mm512_fmadd_ps(in6, _mm512_set1_ps(2.5e-01f), tmp53);
			tmp63 = _mm512_fmadd_ps(in14, _mm512_set1_ps(2.5e-01f), tmp57);
			tmp53 = _mm512_sub_ps(in7, tmp53);
			tmp57 = _mm512_sub_ps(in15, tmp57);
			tmp59 = _mm512_fmadd_ps(in10, _mm512_set1_ps(-1.25e+00f), tmp59);
			tmp63 = _mm512_fmadd_ps(in18, _mm512_set1_ps(-1.25e+00f), tmp63);
			in10 = _mm512_sub_ps(in10, in6);
			in18 = _mm512_sub_ps(in18, in14);
			in10 = _mm512_fmadd_ps(in10, _mm512_set1_ps(5.25e+00f), tmp53);
			in18 = _mm512_fmadd_ps(in18, _mm512_set1_ps(5.25e+00f), tmp57);
			tmp54 = _mm512_fmadd_ps(tmp59, _mm512_set1_ps(2e+00f), tmp52);
			tmp58 = _mm512_fmadd_ps(tmp63, _mm512_set1_ps(2e+00f), tmp56);
			tmp52 = _mm512_fnmadd_ps(tmp59, _mm512_set1_ps(2e+00f), tmp52);
			tmp56 = _mm512_fnmadd_ps(tmp63, _mm512_set1_ps(2e+00f), tmp56);
			__m512 out17 = _mm512_shuffle_f32x4(in4, tmp61, 68);
			__m512 out25 = _mm512_shuffle_f32x4(in4, tmp61, 238);
			__m512 out18 = _mm512_shuffle_f32x4(tmp62, in8, 68);
			__m512 out26 = _mm512_shuffle_f32x4(tmp62, in8, 238);
			__m512 out19 = _mm512_shuffle_f32x4(tmp60, tmp54, 68);
			__m512 out27 = _mm512_shuffle_f32x4(tmp60, tmp54, 238);
			__m512 out20 = _mm512_shuffle_f32x4(tmp52, in10, 68);
			__m512 out28 = _mm512_shuffle_f32x4(tmp52, in10, 238);
			__m512 out21 = _mm512_shuffle_f32x4(in12, tmp65, 68);
			__m512 out29 = _mm512_shuffle_f32x4(in12, tmp65, 238);
			__m512 out22 = _mm512_shuffle_f32x4(tmp66, in16, 68);
			__m512 out30 = _mm512_shuffle_f32x4(tmp66, in16, 238);
			__m512 out23 = _mm512_shuffle_f32x4(tmp64, tmp58, 68);
			__m512 out31 = _mm512_shuffle_f32x4(tmp64, tmp58, 238);
			__m512 out24 = _mm512_shuffle_f32x4(tmp56, in18, 68);
			__m512 out32 = _mm512_shuffle_f32x4(tmp56, in18, 238);
			_mm512_storeu_ps(dfPtr1+0+107520*i6+16128*j2+16128*s2+768*k2, out17);
			_mm512_storeu_ps(dfPtr1+128+107520*i6+16128*j2+16128*s2+768*k2, out25);
			_mm512_storeu_ps(dfPtr1+64+107520*i6+16128*j2+16128*s2+768*k2, out21);
			_mm512_storeu_ps(dfPtr1+192+107520*i6+16128*j2+16128*s2+768*k2, out29);
			_mm512_storeu_ps(dfPtr1+26880+107520*i6+16128*j2+16128*s2+768*k2, out18);
			_mm512_storeu_ps(dfPtr1+27008+107520*i6+16128*j2+16128*s2+768*k2, out26);
			_mm512_storeu_ps(dfPtr1+26944+107520*i6+16128*j2+16128*s2+768*k2, out22);
			_mm512_storeu_ps(dfPtr1+27072+107520*i6+16128*j2+16128*s2+768*k2, out30);
			_mm512_storeu_ps(dfPtr1+53760+107520*i6+16128*j2+16128*s2+768*k2, out19);
			_mm512_storeu_ps(dfPtr1+53888+107520*i6+16128*j2+16128*s2+768*k2, out27);
			_mm512_storeu_ps(dfPtr1+53824+107520*i6+16128*j2+16128*s2+768*k2, out23);
			_mm512_storeu_ps(dfPtr1+53952+107520*i6+16128*j2+16128*s2+768*k2, out31);
			_mm512_storeu_ps(dfPtr1+80640+107520*i6+16128*j2+16128*s2+768*k2, out20);
			_mm512_storeu_ps(dfPtr1+80768+107520*i6+16128*j2+16128*s2+768*k2, out28);
			_mm512_storeu_ps(dfPtr1+80704+107520*i6+16128*j2+16128*s2+768*k2, out24);
			_mm512_storeu_ps(dfPtr1+80832+107520*i6+16128*j2+16128*s2+768*k2, out32);
			__m512 dat17 = _mm512_maskz_loadu_ps(255, datPtr1+96+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat17 = _mm512_mask_fmadd_ps(dat17, 255, bnMul1, bnAdd1);
			__mmask16 mask19 = _mm512_cmp_ps_mask(dat17, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat17 = _mm512_mask_mul_ps(dat17, mask19, dat17, _mm512_set1_ps(6.25e-01f));
			dat17 = _mm512_add_ps(dat17, _mm512_maskz_loadu_ps(255, datPtr2+96+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat18 = _mm512_maskz_loadu_ps(255, datPtr1+768+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat18 = _mm512_mask_fmadd_ps(dat18, 255, bnMul1, bnAdd1);
			__mmask16 mask20 = _mm512_cmp_ps_mask(dat18, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat18 = _mm512_mask_mul_ps(dat18, mask20, dat18, _mm512_set1_ps(6.25e-01f));
			dat18 = _mm512_add_ps(dat18, _mm512_maskz_loadu_ps(255, datPtr2+768+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat19 = _mm512_maskz_loadu_ps(16383, datPtr1+1664+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			__m512 bnMul2 = _mm512_set1_ps(((float*)bnPtr3+(ptrdiff_t)2*(1+42*i6+42*s2+2*k2))[0]);
			__m512 bnAdd2 = _mm512_set1_ps(((float*)bnPtr3+(ptrdiff_t)2*(1+42*i6+42*s2+2*k2))[1]);
			dat19 = _mm512_mask_fmadd_ps(dat19, 16383, bnMul2, bnAdd2);
			__mmask16 mask21 = _mm512_cmp_ps_mask(dat19, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat19 = _mm512_mask_mul_ps(dat19, mask21, dat19, _mm512_set1_ps(6.25e-01f));
			dat19 = _mm512_add_ps(dat19, _mm512_maskz_loadu_ps(16383, datPtr2+1664+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512i pm4 = _mm512_set_epi32(23, 22, 21, 20, 19, 18, 17, 16, 7, 6, 5, 4, 3, 2, 1, 0);
			__m512 in20 = _mm512_permutex2var_ps(dat17, pm4, dat18);
			__m512i pm5 = _mm512_set_epi32(13, 12, 11, 10, 9, 8, 7, 6, 7, 6, 5, 4, 3, 2, 1, 0);
			__m512 in28 = _mm512_permutexvar_ps(pm5, dat19);
			__m512 dat20 = _mm512_maskz_loadu_ps(255, datPtr1+224+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat20 = _mm512_mask_fmadd_ps(dat20, 255, bnMul1, bnAdd1);
			__mmask16 mask22 = _mm512_cmp_ps_mask(dat20, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat20 = _mm512_mask_mul_ps(dat20, mask22, dat20, _mm512_set1_ps(6.25e-01f));
			dat20 = _mm512_add_ps(dat20, _mm512_maskz_loadu_ps(255, datPtr2+224+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat21 = _mm512_maskz_loadu_ps(255, datPtr1+896+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat21 = _mm512_mask_fmadd_ps(dat21, 255, bnMul1, bnAdd1);
			__mmask16 mask23 = _mm512_cmp_ps_mask(dat21, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat21 = _mm512_mask_mul_ps(dat21, mask23, dat21, _mm512_set1_ps(6.25e-01f));
			dat21 = _mm512_add_ps(dat21, _mm512_maskz_loadu_ps(255, datPtr2+896+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat22 = _mm512_maskz_loadu_ps(16383, datPtr1+1792+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat22 = _mm512_mask_fmadd_ps(dat22, 16383, bnMul2, bnAdd2);
			__mmask16 mask24 = _mm512_cmp_ps_mask(dat22, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat22 = _mm512_mask_mul_ps(dat22, mask24, dat22, _mm512_set1_ps(6.25e-01f));
			dat22 = _mm512_add_ps(dat22, _mm512_maskz_loadu_ps(16383, datPtr2+1792+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in21 = _mm512_permutex2var_ps(dat20, pm4, dat21);
			__m512 in29 = _mm512_permutexvar_ps(pm5, dat22);
			__m512 dat23 = _mm512_maskz_loadu_ps(255, datPtr1+352+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat23 = _mm512_mask_fmadd_ps(dat23, 255, bnMul1, bnAdd1);
			__mmask16 mask25 = _mm512_cmp_ps_mask(dat23, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat23 = _mm512_mask_mul_ps(dat23, mask25, dat23, _mm512_set1_ps(6.25e-01f));
			dat23 = _mm512_add_ps(dat23, _mm512_maskz_loadu_ps(255, datPtr2+352+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat24 = _mm512_maskz_loadu_ps(255, datPtr1+1024+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat24 = _mm512_mask_fmadd_ps(dat24, 255, bnMul1, bnAdd1);
			__mmask16 mask26 = _mm512_cmp_ps_mask(dat24, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat24 = _mm512_mask_mul_ps(dat24, mask26, dat24, _mm512_set1_ps(6.25e-01f));
			dat24 = _mm512_add_ps(dat24, _mm512_maskz_loadu_ps(255, datPtr2+1024+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat25 = _mm512_maskz_loadu_ps(16383, datPtr1+1920+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat25 = _mm512_mask_fmadd_ps(dat25, 16383, bnMul2, bnAdd2);
			__mmask16 mask27 = _mm512_cmp_ps_mask(dat25, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat25 = _mm512_mask_mul_ps(dat25, mask27, dat25, _mm512_set1_ps(6.25e-01f));
			dat25 = _mm512_add_ps(dat25, _mm512_maskz_loadu_ps(16383, datPtr2+1920+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in22 = _mm512_permutex2var_ps(dat23, pm4, dat24);
			__m512 in30 = _mm512_permutexvar_ps(pm5, dat25);
			__m512 dat26 = _mm512_maskz_loadu_ps(255, datPtr1+480+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat26 = _mm512_mask_fmadd_ps(dat26, 255, bnMul1, bnAdd1);
			__mmask16 mask28 = _mm512_cmp_ps_mask(dat26, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat26 = _mm512_mask_mul_ps(dat26, mask28, dat26, _mm512_set1_ps(6.25e-01f));
			dat26 = _mm512_add_ps(dat26, _mm512_maskz_loadu_ps(255, datPtr2+480+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat27 = _mm512_maskz_loadu_ps(255, datPtr1+1152+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat27 = _mm512_mask_fmadd_ps(dat27, 255, bnMul1, bnAdd1);
			__mmask16 mask29 = _mm512_cmp_ps_mask(dat27, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat27 = _mm512_mask_mul_ps(dat27, mask29, dat27, _mm512_set1_ps(6.25e-01f));
			dat27 = _mm512_add_ps(dat27, _mm512_maskz_loadu_ps(255, datPtr2+1152+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat28 = _mm512_maskz_loadu_ps(16383, datPtr1+2048+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat28 = _mm512_mask_fmadd_ps(dat28, 16383, bnMul2, bnAdd2);
			__mmask16 mask30 = _mm512_cmp_ps_mask(dat28, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat28 = _mm512_mask_mul_ps(dat28, mask30, dat28, _mm512_set1_ps(6.25e-01f));
			dat28 = _mm512_add_ps(dat28, _mm512_maskz_loadu_ps(16383, datPtr2+2048+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in23 = _mm512_permutex2var_ps(dat26, pm4, dat27);
			__m512 in31 = _mm512_permutexvar_ps(pm5, dat28);
			__m512 dat29 = _mm512_maskz_loadu_ps(255, datPtr1+608+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat29 = _mm512_mask_fmadd_ps(dat29, 255, bnMul1, bnAdd1);
			__mmask16 mask31 = _mm512_cmp_ps_mask(dat29, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat29 = _mm512_mask_mul_ps(dat29, mask31, dat29, _mm512_set1_ps(6.25e-01f));
			dat29 = _mm512_add_ps(dat29, _mm512_maskz_loadu_ps(255, datPtr2+608+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat30 = _mm512_maskz_loadu_ps(255, datPtr1+1280+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat30 = _mm512_mask_fmadd_ps(dat30, 255, bnMul1, bnAdd1);
			__mmask16 mask32 = _mm512_cmp_ps_mask(dat30, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat30 = _mm512_mask_mul_ps(dat30, mask32, dat30, _mm512_set1_ps(6.25e-01f));
			dat30 = _mm512_add_ps(dat30, _mm512_maskz_loadu_ps(255, datPtr2+1280+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat31 = _mm512_maskz_loadu_ps(16383, datPtr1+2176+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat31 = _mm512_mask_fmadd_ps(dat31, 16383, bnMul2, bnAdd2);
			__mmask16 mask33 = _mm512_cmp_ps_mask(dat31, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat31 = _mm512_mask_mul_ps(dat31, mask33, dat31, _mm512_set1_ps(6.25e-01f));
			dat31 = _mm512_add_ps(dat31, _mm512_maskz_loadu_ps(16383, datPtr2+2176+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in24 = _mm512_permutex2var_ps(dat29, pm4, dat30);
			__m512 in32 = _mm512_permutexvar_ps(pm5, dat31);
			__m512 dat32 = _mm512_maskz_loadu_ps(255, datPtr1+736+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat32 = _mm512_mask_fmadd_ps(dat32, 255, bnMul1, bnAdd1);
			__mmask16 mask34 = _mm512_cmp_ps_mask(dat32, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat32 = _mm512_mask_mul_ps(dat32, mask34, dat32, _mm512_set1_ps(6.25e-01f));
			dat32 = _mm512_add_ps(dat32, _mm512_maskz_loadu_ps(255, datPtr2+736+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat33 = _mm512_maskz_loadu_ps(255, datPtr1+1408+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat33 = _mm512_mask_fmadd_ps(dat33, 255, bnMul1, bnAdd1);
			__mmask16 mask35 = _mm512_cmp_ps_mask(dat33, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat33 = _mm512_mask_mul_ps(dat33, mask35, dat33, _mm512_set1_ps(6.25e-01f));
			dat33 = _mm512_add_ps(dat33, _mm512_maskz_loadu_ps(255, datPtr2+1408+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat34 = _mm512_maskz_loadu_ps(16383, datPtr1+2304+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat34 = _mm512_mask_fmadd_ps(dat34, 16383, bnMul2, bnAdd2);
			__mmask16 mask36 = _mm512_cmp_ps_mask(dat34, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat34 = _mm512_mask_mul_ps(dat34, mask36, dat34, _mm512_set1_ps(6.25e-01f));
			dat34 = _mm512_add_ps(dat34, _mm512_maskz_loadu_ps(16383, datPtr2+2304+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in25 = _mm512_permutex2var_ps(dat32, pm4, dat33);
			__m512 in33 = _mm512_permutexvar_ps(pm5, dat34);
			__m512 dat35 = _mm512_maskz_loadu_ps(255, datPtr1+864+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat35 = _mm512_mask_fmadd_ps(dat35, 255, bnMul1, bnAdd1);
			__mmask16 mask37 = _mm512_cmp_ps_mask(dat35, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat35 = _mm512_mask_mul_ps(dat35, mask37, dat35, _mm512_set1_ps(6.25e-01f));
			dat35 = _mm512_add_ps(dat35, _mm512_maskz_loadu_ps(255, datPtr2+864+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat36 = _mm512_maskz_loadu_ps(255, datPtr1+1536+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat36 = _mm512_mask_fmadd_ps(dat36, 255, bnMul1, bnAdd1);
			__mmask16 mask38 = _mm512_cmp_ps_mask(dat36, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat36 = _mm512_mask_mul_ps(dat36, mask38, dat36, _mm512_set1_ps(6.25e-01f));
			dat36 = _mm512_add_ps(dat36, _mm512_maskz_loadu_ps(255, datPtr2+1536+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat37 = _mm512_maskz_loadu_ps(16383, datPtr1+2432+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat37 = _mm512_mask_fmadd_ps(dat37, 16383, bnMul2, bnAdd2);
			__mmask16 mask39 = _mm512_cmp_ps_mask(dat37, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat37 = _mm512_mask_mul_ps(dat37, mask39, dat37, _mm512_set1_ps(6.25e-01f));
			dat37 = _mm512_add_ps(dat37, _mm512_maskz_loadu_ps(16383, datPtr2+2432+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in26 = _mm512_permutex2var_ps(dat35, pm4, dat36);
			__m512 in34 = _mm512_permutexvar_ps(pm5, dat37);
			__m512 dat38 = _mm512_maskz_loadu_ps(255, datPtr1+992+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat38 = _mm512_mask_fmadd_ps(dat38, 255, bnMul1, bnAdd1);
			__mmask16 mask40 = _mm512_cmp_ps_mask(dat38, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat38 = _mm512_mask_mul_ps(dat38, mask40, dat38, _mm512_set1_ps(6.25e-01f));
			dat38 = _mm512_add_ps(dat38, _mm512_maskz_loadu_ps(255, datPtr2+992+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat39 = _mm512_maskz_loadu_ps(16383, datPtr1+2560+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat39 = _mm512_mask_fmadd_ps(dat39, 16383, bnMul2, bnAdd2);
			__mmask16 mask41 = _mm512_cmp_ps_mask(dat39, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat39 = _mm512_mask_mul_ps(dat39, mask41, dat39, _mm512_set1_ps(6.25e-01f));
			dat39 = _mm512_add_ps(dat39, _mm512_maskz_loadu_ps(16383, datPtr2+2560+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512i pm6 = _mm512_set_epi32(15, 15, 15, 15, 15, 15, 15, 15, 7, 6, 5, 4, 3, 2, 1, 0);
			__m512 in27 = _mm512_permutexvar_ps(pm6, dat38);
			__m512 in35 = _mm512_permutexvar_ps(pm5, dat39);
			__m512 tmp115 = _mm512_add_ps(in21, in25);
			__m512 tmp119 = _mm512_add_ps(in29, in33);
			__m512 tmp116 = _mm512_sub_ps(in24, in22);
			__m512 tmp120 = _mm512_sub_ps(in32, in30);
			__m512 tmp117 = _mm512_add_ps(in22, in26);
			__m512 tmp121 = _mm512_add_ps(in30, in34);
			in20 = _mm512_sub_ps(in20, in26);
			in28 = _mm512_sub_ps(in28, in34);
			tmp115 = _mm512_fmadd_ps(in23, _mm512_set1_ps(-4.25e+00f), tmp115);
			tmp119 = _mm512_fmadd_ps(in31, _mm512_set1_ps(-4.25e+00f), tmp119);
			tmp117 = _mm512_fmadd_ps(in24, _mm512_set1_ps(-4.25e+00f), tmp117);
			tmp121 = _mm512_fmadd_ps(in32, _mm512_set1_ps(-4.25e+00f), tmp121);
			in20 = _mm512_fmadd_ps(tmp116, _mm512_set1_ps(5.25e+00f), in20);
			in28 = _mm512_fmadd_ps(tmp120, _mm512_set1_ps(5.25e+00f), in28);
			tmp116 = _mm512_fmadd_ps(in22, _mm512_set1_ps(2.5e-01f), in26);
			tmp120 = _mm512_fmadd_ps(in30, _mm512_set1_ps(2.5e-01f), in34);
			in22 = _mm512_fmadd_ps(in22, _mm512_set1_ps(4e+00f), in26);
			in30 = _mm512_fmadd_ps(in30, _mm512_set1_ps(4e+00f), in34);
			__m512 tmp118 = _mm512_sub_ps(tmp117, tmp115);
			__m512 tmp122 = _mm512_sub_ps(tmp121, tmp119);
			tmp117 = _mm512_add_ps(tmp115, tmp117);
			tmp121 = _mm512_add_ps(tmp119, tmp121);
			tmp115 = _mm512_fmadd_ps(in21, _mm512_set1_ps(2.5e-01f), in25);
			tmp119 = _mm512_fmadd_ps(in29, _mm512_set1_ps(2.5e-01f), in33);
			tmp116 = _mm512_fmadd_ps(in24, _mm512_set1_ps(-1.25e+00f), tmp116);
			tmp120 = _mm512_fmadd_ps(in32, _mm512_set1_ps(-1.25e+00f), tmp120);
			in24 = _mm512_fmadd_ps(in24, _mm512_set1_ps(-5e+00f), in22);
			in32 = _mm512_fmadd_ps(in32, _mm512_set1_ps(-5e+00f), in30);
			tmp115 = _mm512_fmadd_ps(in23, _mm512_set1_ps(-1.25e+00f), tmp115);
			tmp119 = _mm512_fmadd_ps(in31, _mm512_set1_ps(-1.25e+00f), tmp119);
			in26 = _mm512_fmadd_ps(tmp115, _mm512_set1_ps(2e+00f), tmp116);
			in34 = _mm512_fmadd_ps(tmp119, _mm512_set1_ps(2e+00f), tmp120);
			tmp116 = _mm512_fnmadd_ps(tmp115, _mm512_set1_ps(2e+00f), tmp116);
			tmp120 = _mm512_fnmadd_ps(tmp119, _mm512_set1_ps(2e+00f), tmp120);
			tmp115 = _mm512_fmadd_ps(in25, _mm512_set1_ps(2.5e-01f), in21);
			tmp119 = _mm512_fmadd_ps(in33, _mm512_set1_ps(2.5e-01f), in29);
			in21 = _mm512_sub_ps(in27, in21);
			in29 = _mm512_sub_ps(in35, in29);
			tmp115 = _mm512_fmadd_ps(in23, _mm512_set1_ps(-1.25e+00f), tmp115);
			tmp119 = _mm512_fmadd_ps(in31, _mm512_set1_ps(-1.25e+00f), tmp119);
			in23 = _mm512_sub_ps(in23, in25);
			in31 = _mm512_sub_ps(in31, in33);
			in23 = _mm512_fmadd_ps(in23, _mm512_set1_ps(5.25e+00f), in21);
			in31 = _mm512_fmadd_ps(in31, _mm512_set1_ps(5.25e+00f), in29);
			in22 = _mm512_fmadd_ps(tmp115, _mm512_set1_ps(2e+00f), in24);
			in30 = _mm512_fmadd_ps(tmp119, _mm512_set1_ps(2e+00f), in32);
			in24 = _mm512_fnmadd_ps(tmp115, _mm512_set1_ps(2e+00f), in24);
			in32 = _mm512_fnmadd_ps(tmp119, _mm512_set1_ps(2e+00f), in32);
			__m512 tmp131 = _mm512_unpacklo_ps(in20, tmp117);
			__m512 tmp132 = _mm512_unpackhi_ps(in20, tmp117);
			__m512 tmp133 = _mm512_unpacklo_ps(tmp118, in26);
			__m512 tmp134 = _mm512_unpackhi_ps(tmp118, in26);
			__m512 tmp135 = _mm512_unpacklo_ps(tmp116, in22);
			__m512 tmp136 = _mm512_unpackhi_ps(tmp116, in22);
			__m512 tmp137 = _mm512_unpacklo_ps(in24, in23);
			__m512 tmp138 = _mm512_unpackhi_ps(in24, in23);
			__m512 tmp139 = _mm512_unpacklo_ps(in28, tmp121);
			__m512 tmp140 = _mm512_unpackhi_ps(in28, tmp121);
			__m512 tmp141 = _mm512_unpacklo_ps(tmp122, in34);
			__m512 tmp142 = _mm512_unpackhi_ps(tmp122, in34);
			__m512 tmp143 = _mm512_unpacklo_ps(tmp120, in30);
			__m512 tmp144 = _mm512_unpackhi_ps(tmp120, in30);
			__m512 tmp145 = _mm512_unpacklo_ps(in32, in31);
			__m512 tmp146 = _mm512_unpackhi_ps(in32, in31);
			__m512 tmp147 = _mm512_shuffle_ps(tmp131, tmp133, 68);
			__m512 tmp148 = _mm512_shuffle_ps(tmp131, tmp133, 238);
			__m512 tmp149 = _mm512_shuffle_ps(tmp132, tmp134, 68);
			__m512 tmp150 = _mm512_shuffle_ps(tmp132, tmp134, 238);
			__m512 tmp151 = _mm512_shuffle_ps(tmp135, tmp137, 68);
			__m512 tmp152 = _mm512_shuffle_ps(tmp135, tmp137, 238);
			__m512 tmp153 = _mm512_shuffle_ps(tmp136, tmp138, 68);
			__m512 tmp154 = _mm512_shuffle_ps(tmp136, tmp138, 238);
			__m512 tmp155 = _mm512_shuffle_ps(tmp139, tmp141, 68);
			__m512 tmp156 = _mm512_shuffle_ps(tmp139, tmp141, 238);
			__m512 tmp157 = _mm512_shuffle_ps(tmp140, tmp142, 68);
			__m512 tmp158 = _mm512_shuffle_ps(tmp140, tmp142, 238);
			__m512 tmp159 = _mm512_shuffle_ps(tmp143, tmp145, 68);
			__m512 tmp160 = _mm512_shuffle_ps(tmp143, tmp145, 238);
			__m512 tmp161 = _mm512_shuffle_ps(tmp144, tmp146, 68);
			__m512 tmp162 = _mm512_shuffle_ps(tmp144, tmp146, 238);
			__m512 tmp163 = _mm512_shuffle_f32x4(tmp147, tmp151, 136);
			__m512 tmp164 = _mm512_shuffle_f32x4(tmp147, tmp151, 221);
			__m512 tmp165 = _mm512_shuffle_f32x4(tmp148, tmp152, 136);
			__m512 tmp166 = _mm512_shuffle_f32x4(tmp148, tmp152, 221);
			__m512 tmp167 = _mm512_shuffle_f32x4(tmp149, tmp153, 136);
			__m512 tmp168 = _mm512_shuffle_f32x4(tmp149, tmp153, 221);
			__m512 tmp169 = _mm512_shuffle_f32x4(tmp150, tmp154, 136);
			__m512 tmp170 = _mm512_shuffle_f32x4(tmp150, tmp154, 221);
			__m512 tmp171 = _mm512_shuffle_f32x4(tmp155, tmp159, 136);
			__m512 tmp172 = _mm512_shuffle_f32x4(tmp155, tmp159, 221);
			__m512 tmp173 = _mm512_shuffle_f32x4(tmp156, tmp160, 136);
			__m512 tmp174 = _mm512_shuffle_f32x4(tmp156, tmp160, 221);
			__m512 tmp175 = _mm512_shuffle_f32x4(tmp157, tmp161, 136);
			__m512 tmp176 = _mm512_shuffle_f32x4(tmp157, tmp161, 221);
			__m512 tmp177 = _mm512_shuffle_f32x4(tmp158, tmp162, 136);
			__m512 tmp178 = _mm512_shuffle_f32x4(tmp158, tmp162, 221);
			in20 = _mm512_shuffle_f32x4(tmp163, tmp171, 136);
			in28 = _mm512_shuffle_f32x4(tmp163, tmp171, 221);
			tmp117 = _mm512_shuffle_f32x4(tmp165, tmp173, 136);
			tmp121 = _mm512_shuffle_f32x4(tmp165, tmp173, 221);
			tmp118 = _mm512_shuffle_f32x4(tmp167, tmp175, 136);
			tmp122 = _mm512_shuffle_f32x4(tmp167, tmp175, 221);
			in26 = _mm512_shuffle_f32x4(tmp169, tmp177, 136);
			in34 = _mm512_shuffle_f32x4(tmp169, tmp177, 221);
			tmp116 = _mm512_shuffle_f32x4(tmp164, tmp172, 136);
			tmp120 = _mm512_shuffle_f32x4(tmp164, tmp172, 221);
			in22 = _mm512_shuffle_f32x4(tmp166, tmp174, 136);
			in30 = _mm512_shuffle_f32x4(tmp166, tmp174, 221);
			in24 = _mm512_shuffle_f32x4(tmp168, tmp176, 136);
			in32 = _mm512_shuffle_f32x4(tmp168, tmp176, 221);
			in23 = _mm512_shuffle_f32x4(tmp170, tmp178, 136);
			in31 = _mm512_shuffle_f32x4(tmp170, tmp178, 221);
			__m512 tmp123 = _mm512_add_ps(tmp117, in22);
			__m512 tmp127 = _mm512_add_ps(tmp121, in30);
			__m512 tmp124 = _mm512_sub_ps(tmp116, tmp118);
			__m512 tmp128 = _mm512_sub_ps(tmp120, tmp122);
			__m512 tmp125 = _mm512_add_ps(tmp118, in24);
			__m512 tmp129 = _mm512_add_ps(tmp122, in32);
			in20 = _mm512_sub_ps(in20, in24);
			in28 = _mm512_sub_ps(in28, in32);
			tmp123 = _mm512_fmadd_ps(in26, _mm512_set1_ps(-4.25e+00f), tmp123);
			tmp127 = _mm512_fmadd_ps(in34, _mm512_set1_ps(-4.25e+00f), tmp127);
			tmp125 = _mm512_fmadd_ps(tmp116, _mm512_set1_ps(-4.25e+00f), tmp125);
			tmp129 = _mm512_fmadd_ps(tmp120, _mm512_set1_ps(-4.25e+00f), tmp129);
			in20 = _mm512_fmadd_ps(tmp124, _mm512_set1_ps(5.25e+00f), in20);
			in28 = _mm512_fmadd_ps(tmp128, _mm512_set1_ps(5.25e+00f), in28);
			tmp124 = _mm512_fmadd_ps(tmp118, _mm512_set1_ps(2.5e-01f), in24);
			tmp128 = _mm512_fmadd_ps(tmp122, _mm512_set1_ps(2.5e-01f), in32);
			tmp118 = _mm512_fmadd_ps(tmp118, _mm512_set1_ps(4e+00f), in24);
			tmp122 = _mm512_fmadd_ps(tmp122, _mm512_set1_ps(4e+00f), in32);
			__m512 tmp126 = _mm512_sub_ps(tmp125, tmp123);
			__m512 tmp130 = _mm512_sub_ps(tmp129, tmp127);
			tmp125 = _mm512_add_ps(tmp123, tmp125);
			tmp129 = _mm512_add_ps(tmp127, tmp129);
			tmp123 = _mm512_fmadd_ps(tmp117, _mm512_set1_ps(2.5e-01f), in22);
			tmp127 = _mm512_fmadd_ps(tmp121, _mm512_set1_ps(2.5e-01f), in30);
			tmp124 = _mm512_fmadd_ps(tmp116, _mm512_set1_ps(-1.25e+00f), tmp124);
			tmp128 = _mm512_fmadd_ps(tmp120, _mm512_set1_ps(-1.25e+00f), tmp128);
			tmp116 = _mm512_fmadd_ps(tmp116, _mm512_set1_ps(-5e+00f), tmp118);
			tmp120 = _mm512_fmadd_ps(tmp120, _mm512_set1_ps(-5e+00f), tmp122);
			tmp123 = _mm512_fmadd_ps(in26, _mm512_set1_ps(-1.25e+00f), tmp123);
			tmp127 = _mm512_fmadd_ps(in34, _mm512_set1_ps(-1.25e+00f), tmp127);
			in24 = _mm512_fmadd_ps(tmp123, _mm512_set1_ps(2e+00f), tmp124);
			in32 = _mm512_fmadd_ps(tmp127, _mm512_set1_ps(2e+00f), tmp128);
			tmp124 = _mm512_fnmadd_ps(tmp123, _mm512_set1_ps(2e+00f), tmp124);
			tmp128 = _mm512_fnmadd_ps(tmp127, _mm512_set1_ps(2e+00f), tmp128);
			tmp123 = _mm512_fmadd_ps(in22, _mm512_set1_ps(2.5e-01f), tmp117);
			tmp127 = _mm512_fmadd_ps(in30, _mm512_set1_ps(2.5e-01f), tmp121);
			tmp117 = _mm512_sub_ps(in23, tmp117);
			tmp121 = _mm512_sub_ps(in31, tmp121);
			tmp123 = _mm512_fmadd_ps(in26, _mm512_set1_ps(-1.25e+00f), tmp123);
			tmp127 = _mm512_fmadd_ps(in34, _mm512_set1_ps(-1.25e+00f), tmp127);
			in26 = _mm512_sub_ps(in26, in22);
			in34 = _mm512_sub_ps(in34, in30);
			in26 = _mm512_fmadd_ps(in26, _mm512_set1_ps(5.25e+00f), tmp117);
			in34 = _mm512_fmadd_ps(in34, _mm512_set1_ps(5.25e+00f), tmp121);
			tmp118 = _mm512_fmadd_ps(tmp123, _mm512_set1_ps(2e+00f), tmp116);
			tmp122 = _mm512_fmadd_ps(tmp127, _mm512_set1_ps(2e+00f), tmp120);
			tmp116 = _mm512_fnmadd_ps(tmp123, _mm512_set1_ps(2e+00f), tmp116);
			tmp120 = _mm512_fnmadd_ps(tmp127, _mm512_set1_ps(2e+00f), tmp120);
			__m512 out33 = _mm512_shuffle_f32x4(in20, tmp125, 68);
			__m512 out41 = _mm512_shuffle_f32x4(in20, tmp125, 238);
			__m512 out34 = _mm512_shuffle_f32x4(tmp126, in24, 68);
			__m512 out42 = _mm512_shuffle_f32x4(tmp126, in24, 238);
			__m512 out35 = _mm512_shuffle_f32x4(tmp124, tmp118, 68);
			__m512 out43 = _mm512_shuffle_f32x4(tmp124, tmp118, 238);
			__m512 out36 = _mm512_shuffle_f32x4(tmp116, in26, 68);
			__m512 out44 = _mm512_shuffle_f32x4(tmp116, in26, 238);
			__m512 out37 = _mm512_shuffle_f32x4(in28, tmp129, 68);
			__m512 out45 = _mm512_shuffle_f32x4(in28, tmp129, 238);
			__m512 out38 = _mm512_shuffle_f32x4(tmp130, in32, 68);
			__m512 out46 = _mm512_shuffle_f32x4(tmp130, in32, 238);
			__m512 out39 = _mm512_shuffle_f32x4(tmp128, tmp122, 68);
			__m512 out47 = _mm512_shuffle_f32x4(tmp128, tmp122, 238);
			__m512 out40 = _mm512_shuffle_f32x4(tmp120, in34, 68);
			__m512 out48 = _mm512_shuffle_f32x4(tmp120, in34, 238);
			_mm512_storeu_ps(dfPtr1+256+107520*i6+16128*j2+16128*s2+768*k2, out33);
			_mm512_storeu_ps(dfPtr1+384+107520*i6+16128*j2+16128*s2+768*k2, out41);
			_mm512_storeu_ps(dfPtr1+320+107520*i6+16128*j2+16128*s2+768*k2, out37);
			_mm512_storeu_ps(dfPtr1+448+107520*i6+16128*j2+16128*s2+768*k2, out45);
			_mm512_storeu_ps(dfPtr1+27136+107520*i6+16128*j2+16128*s2+768*k2, out34);
			_mm512_storeu_ps(dfPtr1+27264+107520*i6+16128*j2+16128*s2+768*k2, out42);
			_mm512_storeu_ps(dfPtr1+27200+107520*i6+16128*j2+16128*s2+768*k2, out38);
			_mm512_storeu_ps(dfPtr1+27328+107520*i6+16128*j2+16128*s2+768*k2, out46);
			_mm512_storeu_ps(dfPtr1+54016+107520*i6+16128*j2+16128*s2+768*k2, out35);
			_mm512_storeu_ps(dfPtr1+54144+107520*i6+16128*j2+16128*s2+768*k2, out43);
			_mm512_storeu_ps(dfPtr1+54080+107520*i6+16128*j2+16128*s2+768*k2, out39);
			_mm512_storeu_ps(dfPtr1+54208+107520*i6+16128*j2+16128*s2+768*k2, out47);
			_mm512_storeu_ps(dfPtr1+80896+107520*i6+16128*j2+16128*s2+768*k2, out36);
			_mm512_storeu_ps(dfPtr1+81024+107520*i6+16128*j2+16128*s2+768*k2, out44);
			_mm512_storeu_ps(dfPtr1+80960+107520*i6+16128*j2+16128*s2+768*k2, out40);
			_mm512_storeu_ps(dfPtr1+81088+107520*i6+16128*j2+16128*s2+768*k2, out48);
			__m512 dat40 = _mm512_maskz_loadu_ps(16383, datPtr1+1712+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat40 = _mm512_mask_fmadd_ps(dat40, 16383, bnMul2, bnAdd2);
			__mmask16 mask42 = _mm512_cmp_ps_mask(dat40, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat40 = _mm512_mask_mul_ps(dat40, mask42, dat40, _mm512_set1_ps(6.25e-01f));
			dat40 = _mm512_add_ps(dat40, _mm512_maskz_loadu_ps(16383, datPtr2+1712+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat41 = _mm512_maskz_loadu_ps(255, datPtr1+1760+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat41 = _mm512_mask_fmadd_ps(dat41, 255, bnMul2, bnAdd2);
			__mmask16 mask43 = _mm512_cmp_ps_mask(dat41, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat41 = _mm512_mask_mul_ps(dat41, mask43, dat41, _mm512_set1_ps(6.25e-01f));
			dat41 = _mm512_add_ps(dat41, _mm512_maskz_loadu_ps(255, datPtr2+1760+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat42 = _mm512_maskz_loadu_ps(255, datPtr1+2432+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat42 = _mm512_mask_fmadd_ps(dat42, 255, bnMul2, bnAdd2);
			__mmask16 mask44 = _mm512_cmp_ps_mask(dat42, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat42 = _mm512_mask_mul_ps(dat42, mask44, dat42, _mm512_set1_ps(6.25e-01f));
			dat42 = _mm512_add_ps(dat42, _mm512_maskz_loadu_ps(255, datPtr2+2432+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512i pm7 = _mm512_set_epi32(13, 12, 11, 10, 9, 8, 7, 6, 7, 6, 5, 4, 3, 2, 1, 0);
			__m512 in36 = _mm512_permutexvar_ps(pm7, dat40);
			__m512i pm8 = _mm512_set_epi32(23, 22, 21, 20, 19, 18, 17, 16, 7, 6, 5, 4, 3, 2, 1, 0);
			__m512 in44 = _mm512_permutex2var_ps(dat41, pm8, dat42);
			__m512 dat43 = _mm512_maskz_loadu_ps(16383, datPtr1+1840+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat43 = _mm512_mask_fmadd_ps(dat43, 16383, bnMul2, bnAdd2);
			__mmask16 mask45 = _mm512_cmp_ps_mask(dat43, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat43 = _mm512_mask_mul_ps(dat43, mask45, dat43, _mm512_set1_ps(6.25e-01f));
			dat43 = _mm512_add_ps(dat43, _mm512_maskz_loadu_ps(16383, datPtr2+1840+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat44 = _mm512_maskz_loadu_ps(255, datPtr1+1888+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat44 = _mm512_mask_fmadd_ps(dat44, 255, bnMul2, bnAdd2);
			__mmask16 mask46 = _mm512_cmp_ps_mask(dat44, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat44 = _mm512_mask_mul_ps(dat44, mask46, dat44, _mm512_set1_ps(6.25e-01f));
			dat44 = _mm512_add_ps(dat44, _mm512_maskz_loadu_ps(255, datPtr2+1888+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat45 = _mm512_maskz_loadu_ps(255, datPtr1+2560+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat45 = _mm512_mask_fmadd_ps(dat45, 255, bnMul2, bnAdd2);
			__mmask16 mask47 = _mm512_cmp_ps_mask(dat45, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat45 = _mm512_mask_mul_ps(dat45, mask47, dat45, _mm512_set1_ps(6.25e-01f));
			dat45 = _mm512_add_ps(dat45, _mm512_maskz_loadu_ps(255, datPtr2+2560+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in37 = _mm512_permutexvar_ps(pm7, dat43);
			__m512 in45 = _mm512_permutex2var_ps(dat44, pm8, dat45);
			__m512 dat46 = _mm512_maskz_loadu_ps(16383, datPtr1+1968+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat46 = _mm512_mask_fmadd_ps(dat46, 16383, bnMul2, bnAdd2);
			__mmask16 mask48 = _mm512_cmp_ps_mask(dat46, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat46 = _mm512_mask_mul_ps(dat46, mask48, dat46, _mm512_set1_ps(6.25e-01f));
			dat46 = _mm512_add_ps(dat46, _mm512_maskz_loadu_ps(16383, datPtr2+1968+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat47 = _mm512_maskz_loadu_ps(255, datPtr1+2016+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat47 = _mm512_mask_fmadd_ps(dat47, 255, bnMul2, bnAdd2);
			__mmask16 mask49 = _mm512_cmp_ps_mask(dat47, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat47 = _mm512_mask_mul_ps(dat47, mask49, dat47, _mm512_set1_ps(6.25e-01f));
			dat47 = _mm512_add_ps(dat47, _mm512_maskz_loadu_ps(255, datPtr2+2016+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat48 = _mm512_maskz_loadu_ps(255, datPtr1+2688+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat48 = _mm512_mask_fmadd_ps(dat48, 255, bnMul2, bnAdd2);
			__mmask16 mask50 = _mm512_cmp_ps_mask(dat48, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat48 = _mm512_mask_mul_ps(dat48, mask50, dat48, _mm512_set1_ps(6.25e-01f));
			dat48 = _mm512_add_ps(dat48, _mm512_maskz_loadu_ps(255, datPtr2+2688+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in38 = _mm512_permutexvar_ps(pm7, dat46);
			__m512 in46 = _mm512_permutex2var_ps(dat47, pm8, dat48);
			__m512 dat49 = _mm512_maskz_loadu_ps(16383, datPtr1+2096+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat49 = _mm512_mask_fmadd_ps(dat49, 16383, bnMul2, bnAdd2);
			__mmask16 mask51 = _mm512_cmp_ps_mask(dat49, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat49 = _mm512_mask_mul_ps(dat49, mask51, dat49, _mm512_set1_ps(6.25e-01f));
			dat49 = _mm512_add_ps(dat49, _mm512_maskz_loadu_ps(16383, datPtr2+2096+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat50 = _mm512_maskz_loadu_ps(255, datPtr1+2144+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat50 = _mm512_mask_fmadd_ps(dat50, 255, bnMul2, bnAdd2);
			__mmask16 mask52 = _mm512_cmp_ps_mask(dat50, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat50 = _mm512_mask_mul_ps(dat50, mask52, dat50, _mm512_set1_ps(6.25e-01f));
			dat50 = _mm512_add_ps(dat50, _mm512_maskz_loadu_ps(255, datPtr2+2144+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat51 = _mm512_maskz_loadu_ps(255, datPtr1+2816+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat51 = _mm512_mask_fmadd_ps(dat51, 255, bnMul2, bnAdd2);
			__mmask16 mask53 = _mm512_cmp_ps_mask(dat51, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat51 = _mm512_mask_mul_ps(dat51, mask53, dat51, _mm512_set1_ps(6.25e-01f));
			dat51 = _mm512_add_ps(dat51, _mm512_maskz_loadu_ps(255, datPtr2+2816+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in39 = _mm512_permutexvar_ps(pm7, dat49);
			__m512 in47 = _mm512_permutex2var_ps(dat50, pm8, dat51);
			__m512 dat52 = _mm512_maskz_loadu_ps(16383, datPtr1+2224+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat52 = _mm512_mask_fmadd_ps(dat52, 16383, bnMul2, bnAdd2);
			__mmask16 mask54 = _mm512_cmp_ps_mask(dat52, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat52 = _mm512_mask_mul_ps(dat52, mask54, dat52, _mm512_set1_ps(6.25e-01f));
			dat52 = _mm512_add_ps(dat52, _mm512_maskz_loadu_ps(16383, datPtr2+2224+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat53 = _mm512_maskz_loadu_ps(255, datPtr1+2272+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat53 = _mm512_mask_fmadd_ps(dat53, 255, bnMul2, bnAdd2);
			__mmask16 mask55 = _mm512_cmp_ps_mask(dat53, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat53 = _mm512_mask_mul_ps(dat53, mask55, dat53, _mm512_set1_ps(6.25e-01f));
			dat53 = _mm512_add_ps(dat53, _mm512_maskz_loadu_ps(255, datPtr2+2272+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat54 = _mm512_maskz_loadu_ps(255, datPtr1+2944+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat54 = _mm512_mask_fmadd_ps(dat54, 255, bnMul2, bnAdd2);
			__mmask16 mask56 = _mm512_cmp_ps_mask(dat54, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat54 = _mm512_mask_mul_ps(dat54, mask56, dat54, _mm512_set1_ps(6.25e-01f));
			dat54 = _mm512_add_ps(dat54, _mm512_maskz_loadu_ps(255, datPtr2+2944+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in40 = _mm512_permutexvar_ps(pm7, dat52);
			__m512 in48 = _mm512_permutex2var_ps(dat53, pm8, dat54);
			__m512 dat55 = _mm512_maskz_loadu_ps(16383, datPtr1+2352+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat55 = _mm512_mask_fmadd_ps(dat55, 16383, bnMul2, bnAdd2);
			__mmask16 mask57 = _mm512_cmp_ps_mask(dat55, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat55 = _mm512_mask_mul_ps(dat55, mask57, dat55, _mm512_set1_ps(6.25e-01f));
			dat55 = _mm512_add_ps(dat55, _mm512_maskz_loadu_ps(16383, datPtr2+2352+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat56 = _mm512_maskz_loadu_ps(255, datPtr1+2400+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat56 = _mm512_mask_fmadd_ps(dat56, 255, bnMul2, bnAdd2);
			__mmask16 mask58 = _mm512_cmp_ps_mask(dat56, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat56 = _mm512_mask_mul_ps(dat56, mask58, dat56, _mm512_set1_ps(6.25e-01f));
			dat56 = _mm512_add_ps(dat56, _mm512_maskz_loadu_ps(255, datPtr2+2400+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat57 = _mm512_maskz_loadu_ps(255, datPtr1+3072+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat57 = _mm512_mask_fmadd_ps(dat57, 255, bnMul2, bnAdd2);
			__mmask16 mask59 = _mm512_cmp_ps_mask(dat57, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat57 = _mm512_mask_mul_ps(dat57, mask59, dat57, _mm512_set1_ps(6.25e-01f));
			dat57 = _mm512_add_ps(dat57, _mm512_maskz_loadu_ps(255, datPtr2+3072+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in41 = _mm512_permutexvar_ps(pm7, dat55);
			__m512 in49 = _mm512_permutex2var_ps(dat56, pm8, dat57);
			__m512 dat58 = _mm512_maskz_loadu_ps(16383, datPtr1+2480+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat58 = _mm512_mask_fmadd_ps(dat58, 16383, bnMul2, bnAdd2);
			__mmask16 mask60 = _mm512_cmp_ps_mask(dat58, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat58 = _mm512_mask_mul_ps(dat58, mask60, dat58, _mm512_set1_ps(6.25e-01f));
			dat58 = _mm512_add_ps(dat58, _mm512_maskz_loadu_ps(16383, datPtr2+2480+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat59 = _mm512_maskz_loadu_ps(255, datPtr1+2528+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat59 = _mm512_mask_fmadd_ps(dat59, 255, bnMul2, bnAdd2);
			__mmask16 mask61 = _mm512_cmp_ps_mask(dat59, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat59 = _mm512_mask_mul_ps(dat59, mask61, dat59, _mm512_set1_ps(6.25e-01f));
			dat59 = _mm512_add_ps(dat59, _mm512_maskz_loadu_ps(255, datPtr2+2528+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat60 = _mm512_maskz_loadu_ps(255, datPtr1+3200+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat60 = _mm512_mask_fmadd_ps(dat60, 255, bnMul2, bnAdd2);
			__mmask16 mask62 = _mm512_cmp_ps_mask(dat60, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat60 = _mm512_mask_mul_ps(dat60, mask62, dat60, _mm512_set1_ps(6.25e-01f));
			dat60 = _mm512_add_ps(dat60, _mm512_maskz_loadu_ps(255, datPtr2+3200+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in42 = _mm512_permutexvar_ps(pm7, dat58);
			__m512 in50 = _mm512_permutex2var_ps(dat59, pm8, dat60);
			__m512 dat61 = _mm512_maskz_loadu_ps(16383, datPtr1+2608+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat61 = _mm512_mask_fmadd_ps(dat61, 16383, bnMul2, bnAdd2);
			__mmask16 mask63 = _mm512_cmp_ps_mask(dat61, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat61 = _mm512_mask_mul_ps(dat61, mask63, dat61, _mm512_set1_ps(6.25e-01f));
			dat61 = _mm512_add_ps(dat61, _mm512_maskz_loadu_ps(16383, datPtr2+2608+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 dat62 = _mm512_maskz_loadu_ps(255, datPtr1+2656+69888*i6+128*h1+4*w1+69888*s2+3328*k2);
			dat62 = _mm512_mask_fmadd_ps(dat62, 255, bnMul2, bnAdd2);
			__mmask16 mask64 = _mm512_cmp_ps_mask(dat62, _mm512_setzero_ps(), _CMP_LT_OQ);
			dat62 = _mm512_mask_mul_ps(dat62, mask64, dat62, _mm512_set1_ps(6.25e-01f));
			dat62 = _mm512_add_ps(dat62, _mm512_maskz_loadu_ps(255, datPtr2+2656+69888*i6+128*h1+4*w1+69888*s2+3328*k2));
			__m512 in43 = _mm512_permutexvar_ps(pm7, dat61);
			__m512i pm9 = _mm512_set_epi32(15, 15, 15, 15, 15, 15, 15, 15, 7, 6, 5, 4, 3, 2, 1, 0);
			__m512 in51 = _mm512_permutexvar_ps(pm9, dat62);
			__m512 tmp179 = _mm512_add_ps(in37, in41);
			__m512 tmp183 = _mm512_add_ps(in45, in49);
			__m512 tmp180 = _mm512_sub_ps(in40, in38);
			__m512 tmp184 = _mm512_sub_ps(in48, in46);
			__m512 tmp181 = _mm512_add_ps(in38, in42);
			__m512 tmp185 = _mm512_add_ps(in46, in50);
			in36 = _mm512_sub_ps(in36, in42);
			in44 = _mm512_sub_ps(in44, in50);
			tmp179 = _mm512_fmadd_ps(in39, _mm512_set1_ps(-4.25e+00f), tmp179);
			tmp183 = _mm512_fmadd_ps(in47, _mm512_set1_ps(-4.25e+00f), tmp183);
			tmp181 = _mm512_fmadd_ps(in40, _mm512_set1_ps(-4.25e+00f), tmp181);
			tmp185 = _mm512_fmadd_ps(in48, _mm512_set1_ps(-4.25e+00f), tmp185);
			in36 = _mm512_fmadd_ps(tmp180, _mm512_set1_ps(5.25e+00f), in36);
			in44 = _mm512_fmadd_ps(tmp184, _mm512_set1_ps(5.25e+00f), in44);
			tmp180 = _mm512_fmadd_ps(in38, _mm512_set1_ps(2.5e-01f), in42);
			tmp184 = _mm512_fmadd_ps(in46, _mm512_set1_ps(2.5e-01f), in50);
			in38 = _mm512_fmadd_ps(in38, _mm512_set1_ps(4e+00f), in42);
			in46 = _mm512_fmadd_ps(in46, _mm512_set1_ps(4e+00f), in50);
			__m512 tmp182 = _mm512_sub_ps(tmp181, tmp179);
			__m512 tmp186 = _mm512_sub_ps(tmp185, tmp183);
			tmp181 = _mm512_add_ps(tmp179, tmp181);
			tmp185 = _mm512_add_ps(tmp183, tmp185);
			tmp179 = _mm512_fmadd_ps(in37, _mm512_set1_ps(2.5e-01f), in41);
			tmp183 = _mm512_fmadd_ps(in45, _mm512_set1_ps(2.5e-01f), in49);
			tmp180 = _mm512_fmadd_ps(in40, _mm512_set1_ps(-1.25e+00f), tmp180);
			tmp184 = _mm512_fmadd_ps(in48, _mm512_set1_ps(-1.25e+00f), tmp184);
			in40 = _mm512_fmadd_ps(in40, _mm512_set1_ps(-5e+00f), in38);
			in48 = _mm512_fmadd_ps(in48, _mm512_set1_ps(-5e+00f), in46);
			tmp179 = _mm512_fmadd_ps(in39, _mm512_set1_ps(-1.25e+00f), tmp179);
			tmp183 = _mm512_fmadd_ps(in47, _mm512_set1_ps(-1.25e+00f), tmp183);
			in42 = _mm512_fmadd_ps(tmp179, _mm512_set1_ps(2e+00f), tmp180);
			in50 = _mm512_fmadd_ps(tmp183, _mm512_set1_ps(2e+00f), tmp184);
			tmp180 = _mm512_fnmadd_ps(tmp179, _mm512_set1_ps(2e+00f), tmp180);
			tmp184 = _mm512_fnmadd_ps(tmp183, _mm512_set1_ps(2e+00f), tmp184);
			tmp179 = _mm512_fmadd_ps(in41, _mm512_set1_ps(2.5e-01f), in37);
			tmp183 = _mm512_fmadd_ps(in49, _mm512_set1_ps(2.5e-01f), in45);
			in37 = _mm512_sub_ps(in43, in37);
			in45 = _mm512_sub_ps(in51, in45);
			tmp179 = _mm512_fmadd_ps(in39, _mm512_set1_ps(-1.25e+00f), tmp179);
			tmp183 = _mm512_fmadd_ps(in47, _mm512_set1_ps(-1.25e+00f), tmp183);
			in39 = _mm512_sub_ps(in39, in41);
			in47 = _mm512_sub_ps(in47, in49);
			in39 = _mm512_fmadd_ps(in39, _mm512_set1_ps(5.25e+00f), in37);
			in47 = _mm512_fmadd_ps(in47, _mm512_set1_ps(5.25e+00f), in45);
			in38 = _mm512_fmadd_ps(tmp179, _mm512_set1_ps(2e+00f), in40);
			in46 = _mm512_fmadd_ps(tmp183, _mm512_set1_ps(2e+00f), in48);
			in40 = _mm512_fnmadd_ps(tmp179, _mm512_set1_ps(2e+00f), in40);
			in48 = _mm512_fnmadd_ps(tmp183, _mm512_set1_ps(2e+00f), in48);
			__m512 tmp195 = _mm512_unpacklo_ps(in36, tmp181);
			__m512 tmp196 = _mm512_unpackhi_ps(in36, tmp181);
			__m512 tmp197 = _mm512_unpacklo_ps(tmp182, in42);
			__m512 tmp198 = _mm512_unpackhi_ps(tmp182, in42);
			__m512 tmp199 = _mm512_unpacklo_ps(tmp180, in38);
			__m512 tmp200 = _mm512_unpackhi_ps(tmp180, in38);
			__m512 tmp201 = _mm512_unpacklo_ps(in40, in39);
			__m512 tmp202 = _mm512_unpackhi_ps(in40, in39);
			__m512 tmp203 = _mm512_unpacklo_ps(in44, tmp185);
			__m512 tmp204 = _mm512_unpackhi_ps(in44, tmp185);
			__m512 tmp205 = _mm512_unpacklo_ps(tmp186, in50);
			__m512 tmp206 = _mm512_unpackhi_ps(tmp186, in50);
			__m512 tmp207 = _mm512_unpacklo_ps(tmp184, in46);
			__m512 tmp208 = _mm512_unpackhi_ps(tmp184, in46);
			__m512 tmp209 = _mm512_unpacklo_ps(in48, in47);
			__m512 tmp210 = _mm512_unpackhi_ps(in48, in47);
			__m512 tmp211 = _mm512_shuffle_ps(tmp195, tmp197, 68);
			__m512 tmp212 = _mm512_shuffle_ps(tmp195, tmp197, 238);
			__m512 tmp213 = _mm512_shuffle_ps(tmp196, tmp198, 68);
			__m512 tmp214 = _mm512_shuffle_ps(tmp196, tmp198, 238);
			__m512 tmp215 = _mm512_shuffle_ps(tmp199, tmp201, 68);
			__m512 tmp216 = _mm512_shuffle_ps(tmp199, tmp201, 238);
			__m512 tmp217 = _mm512_shuffle_ps(tmp200, tmp202, 68);
			__m512 tmp218 = _mm512_shuffle_ps(tmp200, tmp202, 238);
			__m512 tmp219 = _mm512_shuffle_ps(tmp203, tmp205, 68);
			__m512 tmp220 = _mm512_shuffle_ps(tmp203, tmp205, 238);
			__m512 tmp221 = _mm512_shuffle_ps(tmp204, tmp206, 68);
			__m512 tmp222 = _mm512_shuffle_ps(tmp204, tmp206, 238);
			__m512 tmp223 = _mm512_shuffle_ps(tmp207, tmp209, 68);
			__m512 tmp224 = _mm512_shuffle_ps(tmp207, tmp209, 238);
			__m512 tmp225 = _mm512_shuffle_ps(tmp208, tmp210, 68);
			__m512 tmp226 = _mm512_shuffle_ps(tmp208, tmp210, 238);
			__m512 tmp227 = _mm512_shuffle_f32x4(tmp211, tmp215, 136);
			__m512 tmp228 = _mm512_shuffle_f32x4(tmp211, tmp215, 221);
			__m512 tmp229 = _mm512_shuffle_f32x4(tmp212, tmp216, 136);
			__m512 tmp230 = _mm512_shuffle_f32x4(tmp212, tmp216, 221);
			__m512 tmp231 = _mm512_shuffle_f32x4(tmp213, tmp217, 136);
			__m512 tmp232 = _mm512_shuffle_f32x4(tmp213, tmp217, 221);
			__m512 tmp233 = _mm512_shuffle_f32x4(tmp214, tmp218, 136);
			__m512 tmp234 = _mm512_shuffle_f32x4(tmp214, tmp218, 221);
			__m512 tmp235 = _mm512_shuffle_f32x4(tmp219, tmp223, 136);
			__m512 tmp236 = _mm512_shuffle_f32x4(tmp219, tmp223, 221);
			__m512 tmp237 = _mm512_shuffle_f32x4(tmp220, tmp224, 136);
			__m512 tmp238 = _mm512_shuffle_f32x4(tmp220, tmp224, 221);
			__m512 tmp239 = _mm512_shuffle_f32x4(tmp221, tmp225, 136);
			__m512 tmp240 = _mm512_shuffle_f32x4(tmp221, tmp225, 221);
			__m512 tmp241 = _mm512_shuffle_f32x4(tmp222, tmp226, 136);
			__m512 tmp242 = _mm512_shuffle_f32x4(tmp222, tmp226, 221);
			in36 = _mm512_shuffle_f32x4(tmp227, tmp235, 136);
			in44 = _mm512_shuffle_f32x4(tmp227, tmp235, 221);
			tmp181 = _mm512_shuffle_f32x4(tmp229, tmp237, 136);
			tmp185 = _mm512_shuffle_f32x4(tmp229, tmp237, 221);
			tmp182 = _mm512_shuffle_f32x4(tmp231, tmp239, 136);
			tmp186 = _mm512_shuffle_f32x4(tmp231, tmp239, 221);
			in42 = _mm512_shuffle_f32x4(tmp233, tmp241, 136);
			in50 = _mm512_shuffle_f32x4(tmp233, tmp241, 221);
			tmp180 = _mm512_shuffle_f32x4(tmp228, tmp236, 136);
			tmp184 = _mm512_shuffle_f32x4(tmp228, tmp236, 221);
			in38 = _mm512_shuffle_f32x4(tmp230, tmp238, 136);
			in46 = _mm512_shuffle_f32x4(tmp230, tmp238, 221);
			in40 = _mm512_shuffle_f32x4(tmp232, tmp240, 136);
			in48 = _mm512_shuffle_f32x4(tmp232, tmp240, 221);
			in39 = _mm512_shuffle_f32x4(tmp234, tmp242, 136);
			in47 = _mm512_shuffle_f32x4(tmp234, tmp242, 221);
			__m512 tmp187 = _mm512_add_ps(tmp181, in38);
			__m512 tmp191 = _mm512_add_ps(tmp185, in46);
			__m512 tmp188 = _mm512_sub_ps(tmp180, tmp182);
			__m512 tmp192 = _mm512_sub_ps(tmp184, tmp186);
			__m512 tmp189 = _mm512_add_ps(tmp182, in40);
			__m512 tmp193 = _mm512_add_ps(tmp186, in48);
			in36 = _mm512_sub_ps(in36, in40);
			in44 = _mm512_sub_ps(in44, in48);
			tmp187 = _mm512_fmadd_ps(in42, _mm512_set1_ps(-4.25e+00f), tmp187);
			tmp191 = _mm512_fmadd_ps(in50, _mm512_set1_ps(-4.25e+00f), tmp191);
			tmp189 = _mm512_fmadd_ps(tmp180, _mm512_set1_ps(-4.25e+00f), tmp189);
			tmp193 = _mm512_fmadd_ps(tmp184, _mm512_set1_ps(-4.25e+00f), tmp193);
			in36 = _mm512_fmadd_ps(tmp188, _mm512_set1_ps(5.25e+00f), in36);
			in44 = _mm512_fmadd_ps(tmp192, _mm512_set1_ps(5.25e+00f), in44);
			tmp188 = _mm512_fmadd_ps(tmp182, _mm512_set1_ps(2.5e-01f), in40);
			tmp192 = _mm512_fmadd_ps(tmp186, _mm512_set1_ps(2.5e-01f), in48);
			tmp182 = _mm512_fmadd_ps(tmp182, _mm512_set1_ps(4e+00f), in40);
			tmp186 = _mm512_fmadd_ps(tmp186, _mm512_set1_ps(4e+00f), in48);
			__m512 tmp190 = _mm512_sub_ps(tmp189, tmp187);
			__m512 tmp194 = _mm512_sub_ps(tmp193, tmp191);
			tmp189 = _mm512_add_ps(tmp187, tmp189);
			tmp193 = _mm512_add_ps(tmp191, tmp193);
			tmp187 = _mm512_fmadd_ps(tmp181, _mm512_set1_ps(2.5e-01f), in38);
			tmp191 = _mm512_fmadd_ps(tmp185, _mm512_set1_ps(2.5e-01f), in46);
			tmp188 = _mm512_fmadd_ps(tmp180, _mm512_set1_ps(-1.25e+00f), tmp188);
			tmp192 = _mm512_fmadd_ps(tmp184, _mm512_set1_ps(-1.25e+00f), tmp192);
			tmp180 = _mm512_fmadd_ps(tmp180, _mm512_set1_ps(-5e+00f), tmp182);
			tmp184 = _mm512_fmadd_ps(tmp184, _mm512_set1_ps(-5e+00f), tmp186);
			tmp187 = _mm512_fmadd_ps(in42, _mm512_set1_ps(-1.25e+00f), tmp187);
			tmp191 = _mm512_fmadd_ps(in50, _mm512_set1_ps(-1.25e+00f), tmp191);
			in40 = _mm512_fmadd_ps(tmp187, _mm512_set1_ps(2e+00f), tmp188);
			in48 = _mm512_fmadd_ps(tmp191, _mm512_set1_ps(2e+00f), tmp192);
			tmp188 = _mm512_fnmadd_ps(tmp187, _mm512_set1_ps(2e+00f), tmp188);
			tmp192 = _mm512_fnmadd_ps(tmp191, _mm512_set1_ps(2e+00f), tmp192);
			tmp187 = _mm512_fmadd_ps(in38, _mm512_set1_ps(2.5e-01f), tmp181);
			tmp191 = _mm512_fmadd_ps(in46, _mm512_set1_ps(2.5e-01f), tmp185);
			tmp181 = _mm512_sub_ps(in39, tmp181);
			tmp185 = _mm512_sub_ps(in47, tmp185);
			tmp187 = _mm512_fmadd_ps(in42, _mm512_set1_ps(-1.25e+00f), tmp187);
			tmp191 = _mm512_fmadd_ps(in50, _mm512_set1_ps(-1.25e+00f), tmp191);
			in42 = _mm512_sub_ps(in42, in38);
			in50 = _mm512_sub_ps(in50, in46);
			in42 = _mm512_fmadd_ps(in42, _mm512_set1_ps(5.25e+00f), tmp181);
			in50 = _mm512_fmadd_ps(in50, _mm512_set1_ps(5.25e+00f), tmp185);
			tmp182 = _mm512_fmadd_ps(tmp187, _mm512_set1_ps(2e+00f), tmp180);
			tmp186 = _mm512_fmadd_ps(tmp191, _mm512_set1_ps(2e+00f), tmp184);
			tmp180 = _mm512_fnmadd_ps(tmp187, _mm512_set1_ps(2e+00f), tmp180);
			tmp184 = _mm512_fnmadd_ps(tmp191, _mm512_set1_ps(2e+00f), tmp184);
			__m512 out49 = _mm512_shuffle_f32x4(in36, tmp189, 68);
			__m512 out57 = _mm512_shuffle_f32x4(in36, tmp189, 238);
			__m512 out50 = _mm512_shuffle_f32x4(tmp190, in40, 68);
			__m512 out58 = _mm512_shuffle_f32x4(tmp190, in40, 238);
			__m512 out51 = _mm512_shuffle_f32x4(tmp188, tmp182, 68);
			__m512 out59 = _mm512_shuffle_f32x4(tmp188, tmp182, 238);
			__m512 out52 = _mm512_shuffle_f32x4(tmp180, in42, 68);
			__m512 out60 = _mm512_shuffle_f32x4(tmp180, in42, 238);
			__m512 out53 = _mm512_shuffle_f32x4(in44, tmp193, 68);
			__m512 out61 = _mm512_shuffle_f32x4(in44, tmp193, 238);
			__m512 out54 = _mm512_shuffle_f32x4(tmp194, in48, 68);
			__m512 out62 = _mm512_shuffle_f32x4(tmp194, in48, 238);
			__m512 out55 = _mm512_shuffle_f32x4(tmp192, tmp186, 68);
			__m512 out63 = _mm512_shuffle_f32x4(tmp192, tmp186, 238);
			__m512 out56 = _mm512_shuffle_f32x4(tmp184, in50, 68);
			__m512 out64 = _mm512_shuffle_f32x4(tmp184, in50, 238);
			_mm512_storeu_ps(dfPtr1+512+107520*i6+16128*j2+16128*s2+768*k2, out49);
			_mm512_storeu_ps(dfPtr1+640+107520*i6+16128*j2+16128*s2+768*k2, out57);
			_mm512_storeu_ps(dfPtr1+576+107520*i6+16128*j2+16128*s2+768*k2, out53);
			_mm512_storeu_ps(dfPtr1+704+107520*i6+16128*j2+16128*s2+768*k2, out61);
			_mm512_storeu_ps(dfPtr1+27392+107520*i6+16128*j2+16128*s2+768*k2, out50);
			_mm512_storeu_ps(dfPtr1+27520+107520*i6+16128*j2+16128*s2+768*k2, out58);
			_mm512_storeu_ps(dfPtr1+27456+107520*i6+16128*j2+16128*s2+768*k2, out54);
			_mm512_storeu_ps(dfPtr1+27584+107520*i6+16128*j2+16128*s2+768*k2, out62);
			_mm512_storeu_ps(dfPtr1+54272+107520*i6+16128*j2+16128*s2+768*k2, out51);
			_mm512_storeu_ps(dfPtr1+54400+107520*i6+16128*j2+16128*s2+768*k2, out59);
			_mm512_storeu_ps(dfPtr1+54336+107520*i6+16128*j2+16128*s2+768*k2, out55);
			_mm512_storeu_ps(dfPtr1+54464+107520*i6+16128*j2+16128*s2+768*k2, out63);
			_mm512_storeu_ps(dfPtr1+81152+107520*i6+16128*j2+16128*s2+768*k2, out52);
			_mm512_storeu_ps(dfPtr1+81280+107520*i6+16128*j2+16128*s2+768*k2, out60);
			_mm512_storeu_ps(dfPtr1+81216+107520*i6+16128*j2+16128*s2+768*k2, out56);
			_mm512_storeu_ps(dfPtr1+81344+107520*i6+16128*j2+16128*s2+768*k2, out64);
		}
		++j2;
		rel1 = 1;
	}
	ptrdiff_t h2 = base1+6;
	ptrdiff_t w2 = 6;
	ptrdiff_t k3 = 0;
	for (; k3 != 42; ++k3) {
		__m512 dat63 = _mm512_maskz_loadu_ps(16383, datPtr1+0+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		__m512 bnMul3 = _mm512_set1_ps(((float*)bnPtr3+(ptrdiff_t)2*(0+42*i6+42*s2+1*k3))[0]);
		__m512 bnAdd3 = _mm512_set1_ps(((float*)bnPtr3+(ptrdiff_t)2*(0+42*i6+42*s2+1*k3))[1]);
		dat63 = _mm512_mask_fmadd_ps(dat63, 16383, bnMul3, bnAdd3);
		__mmask16 mask65 = _mm512_cmp_ps_mask(dat63, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat63 = _mm512_mask_mul_ps(dat63, mask65, dat63, _mm512_set1_ps(6.25e-01f));
		dat63 = _mm512_add_ps(dat63, _mm512_maskz_loadu_ps(16383, datPtr2+0+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 dat64 = _mm512_maskz_loadu_ps(16383, datPtr1+48+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat64 = _mm512_mask_fmadd_ps(dat64, 16383, bnMul3, bnAdd3);
		__mmask16 mask66 = _mm512_cmp_ps_mask(dat64, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat64 = _mm512_mask_mul_ps(dat64, mask66, dat64, _mm512_set1_ps(6.25e-01f));
		dat64 = _mm512_add_ps(dat64, _mm512_maskz_loadu_ps(16383, datPtr2+48+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512i pm10 = _mm512_set_epi32(13, 12, 11, 10, 9, 8, 7, 6, 7, 6, 5, 4, 3, 2, 1, 0);
		__m512 in52 = _mm512_permutexvar_ps(pm10, dat63);
		__m512 in59 = _mm512_permutexvar_ps(pm10, dat64);
		__m512 dat65 = _mm512_maskz_loadu_ps(16383, datPtr1+128+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat65 = _mm512_mask_fmadd_ps(dat65, 16383, bnMul3, bnAdd3);
		__mmask16 mask67 = _mm512_cmp_ps_mask(dat65, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat65 = _mm512_mask_mul_ps(dat65, mask67, dat65, _mm512_set1_ps(6.25e-01f));
		dat65 = _mm512_add_ps(dat65, _mm512_maskz_loadu_ps(16383, datPtr2+128+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 dat66 = _mm512_maskz_loadu_ps(16383, datPtr1+176+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat66 = _mm512_mask_fmadd_ps(dat66, 16383, bnMul3, bnAdd3);
		__mmask16 mask68 = _mm512_cmp_ps_mask(dat66, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat66 = _mm512_mask_mul_ps(dat66, mask68, dat66, _mm512_set1_ps(6.25e-01f));
		dat66 = _mm512_add_ps(dat66, _mm512_maskz_loadu_ps(16383, datPtr2+176+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 in53 = _mm512_permutexvar_ps(pm10, dat65);
		__m512 in60 = _mm512_permutexvar_ps(pm10, dat66);
		__m512 dat67 = _mm512_maskz_loadu_ps(16383, datPtr1+256+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat67 = _mm512_mask_fmadd_ps(dat67, 16383, bnMul3, bnAdd3);
		__mmask16 mask69 = _mm512_cmp_ps_mask(dat67, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat67 = _mm512_mask_mul_ps(dat67, mask69, dat67, _mm512_set1_ps(6.25e-01f));
		dat67 = _mm512_add_ps(dat67, _mm512_maskz_loadu_ps(16383, datPtr2+256+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 dat68 = _mm512_maskz_loadu_ps(16383, datPtr1+304+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat68 = _mm512_mask_fmadd_ps(dat68, 16383, bnMul3, bnAdd3);
		__mmask16 mask70 = _mm512_cmp_ps_mask(dat68, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat68 = _mm512_mask_mul_ps(dat68, mask70, dat68, _mm512_set1_ps(6.25e-01f));
		dat68 = _mm512_add_ps(dat68, _mm512_maskz_loadu_ps(16383, datPtr2+304+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 in54 = _mm512_permutexvar_ps(pm10, dat67);
		__m512 in61 = _mm512_permutexvar_ps(pm10, dat68);
		__m512 dat69 = _mm512_maskz_loadu_ps(16383, datPtr1+384+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat69 = _mm512_mask_fmadd_ps(dat69, 16383, bnMul3, bnAdd3);
		__mmask16 mask71 = _mm512_cmp_ps_mask(dat69, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat69 = _mm512_mask_mul_ps(dat69, mask71, dat69, _mm512_set1_ps(6.25e-01f));
		dat69 = _mm512_add_ps(dat69, _mm512_maskz_loadu_ps(16383, datPtr2+384+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 dat70 = _mm512_maskz_loadu_ps(16383, datPtr1+432+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat70 = _mm512_mask_fmadd_ps(dat70, 16383, bnMul3, bnAdd3);
		__mmask16 mask72 = _mm512_cmp_ps_mask(dat70, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat70 = _mm512_mask_mul_ps(dat70, mask72, dat70, _mm512_set1_ps(6.25e-01f));
		dat70 = _mm512_add_ps(dat70, _mm512_maskz_loadu_ps(16383, datPtr2+432+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 in55 = _mm512_permutexvar_ps(pm10, dat69);
		__m512 in62 = _mm512_permutexvar_ps(pm10, dat70);
		__m512 dat71 = _mm512_maskz_loadu_ps(16383, datPtr1+512+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat71 = _mm512_mask_fmadd_ps(dat71, 16383, bnMul3, bnAdd3);
		__mmask16 mask73 = _mm512_cmp_ps_mask(dat71, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat71 = _mm512_mask_mul_ps(dat71, mask73, dat71, _mm512_set1_ps(6.25e-01f));
		dat71 = _mm512_add_ps(dat71, _mm512_maskz_loadu_ps(16383, datPtr2+512+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 dat72 = _mm512_maskz_loadu_ps(16383, datPtr1+560+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat72 = _mm512_mask_fmadd_ps(dat72, 16383, bnMul3, bnAdd3);
		__mmask16 mask74 = _mm512_cmp_ps_mask(dat72, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat72 = _mm512_mask_mul_ps(dat72, mask74, dat72, _mm512_set1_ps(6.25e-01f));
		dat72 = _mm512_add_ps(dat72, _mm512_maskz_loadu_ps(16383, datPtr2+560+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 in56 = _mm512_permutexvar_ps(pm10, dat71);
		__m512 in63 = _mm512_permutexvar_ps(pm10, dat72);
		__m512 dat73 = _mm512_maskz_loadu_ps(16383, datPtr1+640+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat73 = _mm512_mask_fmadd_ps(dat73, 16383, bnMul3, bnAdd3);
		__mmask16 mask75 = _mm512_cmp_ps_mask(dat73, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat73 = _mm512_mask_mul_ps(dat73, mask75, dat73, _mm512_set1_ps(6.25e-01f));
		dat73 = _mm512_add_ps(dat73, _mm512_maskz_loadu_ps(16383, datPtr2+640+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 dat74 = _mm512_maskz_loadu_ps(16383, datPtr1+688+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat74 = _mm512_mask_fmadd_ps(dat74, 16383, bnMul3, bnAdd3);
		__mmask16 mask76 = _mm512_cmp_ps_mask(dat74, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat74 = _mm512_mask_mul_ps(dat74, mask76, dat74, _mm512_set1_ps(6.25e-01f));
		dat74 = _mm512_add_ps(dat74, _mm512_maskz_loadu_ps(16383, datPtr2+688+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 in57 = _mm512_permutexvar_ps(pm10, dat73);
		__m512 in64 = _mm512_permutexvar_ps(pm10, dat74);
		__m512 dat75 = _mm512_maskz_loadu_ps(16383, datPtr1+768+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat75 = _mm512_mask_fmadd_ps(dat75, 16383, bnMul3, bnAdd3);
		__mmask16 mask77 = _mm512_cmp_ps_mask(dat75, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat75 = _mm512_mask_mul_ps(dat75, mask77, dat75, _mm512_set1_ps(6.25e-01f));
		dat75 = _mm512_add_ps(dat75, _mm512_maskz_loadu_ps(16383, datPtr2+768+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 dat76 = _mm512_maskz_loadu_ps(16383, datPtr1+816+69888*i6+128*h2+4*w2+69888*s2+1664*k3);
		dat76 = _mm512_mask_fmadd_ps(dat76, 16383, bnMul3, bnAdd3);
		__mmask16 mask78 = _mm512_cmp_ps_mask(dat76, _mm512_setzero_ps(), _CMP_LT_OQ);
		dat76 = _mm512_mask_mul_ps(dat76, mask78, dat76, _mm512_set1_ps(6.25e-01f));
		dat76 = _mm512_add_ps(dat76, _mm512_maskz_loadu_ps(16383, datPtr2+816+69888*i6+128*h2+4*w2+69888*s2+1664*k3));
		__m512 in58 = _mm512_permutexvar_ps(pm10, dat75);
		__m512 in65 = _mm512_permutexvar_ps(pm10, dat76);
		__m512 tmp243 = _mm512_add_ps(in53, in57);
		__m512 tmp247 = _mm512_add_ps(in60, in64);
		__m512 tmp244 = _mm512_sub_ps(in56, in54);
		__m512 tmp248 = _mm512_sub_ps(in63, in61);
		__m512 tmp245 = _mm512_add_ps(in54, in58);
		__m512 tmp249 = _mm512_add_ps(in61, in65);
		in52 = _mm512_sub_ps(in52, in58);
		in59 = _mm512_sub_ps(in59, in65);
		tmp243 = _mm512_fmadd_ps(in55, _mm512_set1_ps(-4.25e+00f), tmp243);
		tmp247 = _mm512_fmadd_ps(in62, _mm512_set1_ps(-4.25e+00f), tmp247);
		tmp245 = _mm512_fmadd_ps(in56, _mm512_set1_ps(-4.25e+00f), tmp245);
		tmp249 = _mm512_fmadd_ps(in63, _mm512_set1_ps(-4.25e+00f), tmp249);
		in52 = _mm512_fmadd_ps(tmp244, _mm512_set1_ps(5.25e+00f), in52);
		in59 = _mm512_fmadd_ps(tmp248, _mm512_set1_ps(5.25e+00f), in59);
		tmp244 = _mm512_fmadd_ps(in54, _mm512_set1_ps(2.5e-01f), in58);
		tmp248 = _mm512_fmadd_ps(in61, _mm512_set1_ps(2.5e-01f), in65);
		in54 = _mm512_fmadd_ps(in54, _mm512_set1_ps(4e+00f), in58);
		in61 = _mm512_fmadd_ps(in61, _mm512_set1_ps(4e+00f), in65);
		__m512 tmp246 = _mm512_sub_ps(tmp245, tmp243);
		__m512 tmp250 = _mm512_sub_ps(tmp249, tmp247);
		tmp245 = _mm512_add_ps(tmp243, tmp245);
		tmp249 = _mm512_add_ps(tmp247, tmp249);
		tmp243 = _mm512_fmadd_ps(in53, _mm512_set1_ps(2.5e-01f), in57);
		tmp247 = _mm512_fmadd_ps(in60, _mm512_set1_ps(2.5e-01f), in64);
		tmp244 = _mm512_fmadd_ps(in56, _mm512_set1_ps(-1.25e+00f), tmp244);
		tmp248 = _mm512_fmadd_ps(in63, _mm512_set1_ps(-1.25e+00f), tmp248);
		in56 = _mm512_fmadd_ps(in56, _mm512_set1_ps(-5e+00f), in54);
		in63 = _mm512_fmadd_ps(in63, _mm512_set1_ps(-5e+00f), in61);
		tmp243 = _mm512_fmadd_ps(in55, _mm512_set1_ps(-1.25e+00f), tmp243);
		tmp247 = _mm512_fmadd_ps(in62, _mm512_set1_ps(-1.25e+00f), tmp247);
		in58 = _mm512_fmadd_ps(tmp243, _mm512_set1_ps(2e+00f), tmp244);
		in65 = _mm512_fmadd_ps(tmp247, _mm512_set1_ps(2e+00f), tmp248);
		tmp244 = _mm512_fnmadd_ps(tmp243, _mm512_set1_ps(2e+00f), tmp244);
		tmp248 = _mm512_fnmadd_ps(tmp247, _mm512_set1_ps(2e+00f), tmp248);
		tmp243 = _mm512_fmadd_ps(in57, _mm512_set1_ps(2.5e-01f), in53);
		tmp247 = _mm512_fmadd_ps(in64, _mm512_set1_ps(2.5e-01f), in60);
		in53 = _mm512_sub_ps(_mm512_setzero_ps(), in53);
		in60 = _mm512_sub_ps(_mm512_setzero_ps(), in60);
		tmp243 = _mm512_fmadd_ps(in55, _mm512_set1_ps(-1.25e+00f), tmp243);
		tmp247 = _mm512_fmadd_ps(in62, _mm512_set1_ps(-1.25e+00f), tmp247);
		in55 = _mm512_sub_ps(in55, in57);
		in62 = _mm512_sub_ps(in62, in64);
		in55 = _mm512_fmadd_ps(in55, _mm512_set1_ps(5.25e+00f), in53);
		in62 = _mm512_fmadd_ps(in62, _mm512_set1_ps(5.25e+00f), in60);
		in54 = _mm512_fmadd_ps(tmp243, _mm512_set1_ps(2e+00f), in56);
		in61 = _mm512_fmadd_ps(tmp247, _mm512_set1_ps(2e+00f), in63);
		in56 = _mm512_fnmadd_ps(tmp243, _mm512_set1_ps(2e+00f), in56);
		in63 = _mm512_fnmadd_ps(tmp247, _mm512_set1_ps(2e+00f), in63);
		__m512 tmp259 = _mm512_unpacklo_ps(in52, tmp245);
		__m512 tmp260 = _mm512_unpackhi_ps(in52, tmp245);
		__m512 tmp261 = _mm512_unpacklo_ps(tmp246, in58);
		__m512 tmp262 = _mm512_unpackhi_ps(tmp246, in58);
		__m512 tmp263 = _mm512_unpacklo_ps(tmp244, in54);
		__m512 tmp264 = _mm512_unpackhi_ps(tmp244, in54);
		__m512 tmp265 = _mm512_unpacklo_ps(in56, in55);
		__m512 tmp266 = _mm512_unpackhi_ps(in56, in55);
		__m512 tmp267 = _mm512_unpacklo_ps(in59, tmp249);
		__m512 tmp268 = _mm512_unpackhi_ps(in59, tmp249);
		__m512 tmp269 = _mm512_unpacklo_ps(tmp250, in65);
		__m512 tmp270 = _mm512_unpackhi_ps(tmp250, in65);
		__m512 tmp271 = _mm512_unpacklo_ps(tmp248, in61);
		__m512 tmp272 = _mm512_unpackhi_ps(tmp248, in61);
		__m512 tmp273 = _mm512_unpacklo_ps(in63, in62);
		__m512 tmp274 = _mm512_unpackhi_ps(in63, in62);
		__m512 tmp275 = _mm512_shuffle_ps(tmp259, tmp261, 68);
		__m512 tmp276 = _mm512_shuffle_ps(tmp259, tmp261, 238);
		__m512 tmp277 = _mm512_shuffle_ps(tmp260, tmp262, 68);
		__m512 tmp278 = _mm512_shuffle_ps(tmp260, tmp262, 238);
		__m512 tmp279 = _mm512_shuffle_ps(tmp263, tmp265, 68);
		__m512 tmp280 = _mm512_shuffle_ps(tmp263, tmp265, 238);
		__m512 tmp281 = _mm512_shuffle_ps(tmp264, tmp266, 68);
		__m512 tmp282 = _mm512_shuffle_ps(tmp264, tmp266, 238);
		__m512 tmp283 = _mm512_shuffle_ps(tmp267, tmp269, 68);
		__m512 tmp284 = _mm512_shuffle_ps(tmp267, tmp269, 238);
		__m512 tmp285 = _mm512_shuffle_ps(tmp268, tmp270, 68);
		__m512 tmp286 = _mm512_shuffle_ps(tmp268, tmp270, 238);
		__m512 tmp287 = _mm512_shuffle_ps(tmp271, tmp273, 68);
		__m512 tmp288 = _mm512_shuffle_ps(tmp271, tmp273, 238);
		__m512 tmp289 = _mm512_shuffle_ps(tmp272, tmp274, 68);
		__m512 tmp290 = _mm512_shuffle_ps(tmp272, tmp274, 238);
		__m512 tmp291 = _mm512_shuffle_f32x4(tmp275, tmp279, 136);
		__m512 tmp292 = _mm512_shuffle_f32x4(tmp275, tmp279, 221);
		__m512 tmp293 = _mm512_shuffle_f32x4(tmp276, tmp280, 136);
		__m512 tmp294 = _mm512_shuffle_f32x4(tmp276, tmp280, 221);
		__m512 tmp295 = _mm512_shuffle_f32x4(tmp277, tmp281, 136);
		__m512 tmp296 = _mm512_shuffle_f32x4(tmp277, tmp281, 221);
		__m512 tmp297 = _mm512_shuffle_f32x4(tmp278, tmp282, 136);
		__m512 tmp298 = _mm512_shuffle_f32x4(tmp278, tmp282, 221);
		__m512 tmp299 = _mm512_shuffle_f32x4(tmp283, tmp287, 136);
		__m512 tmp300 = _mm512_shuffle_f32x4(tmp283, tmp287, 221);
		__m512 tmp301 = _mm512_shuffle_f32x4(tmp284, tmp288, 136);
		__m512 tmp302 = _mm512_shuffle_f32x4(tmp284, tmp288, 221);
		__m512 tmp303 = _mm512_shuffle_f32x4(tmp285, tmp289, 136);
		__m512 tmp304 = _mm512_shuffle_f32x4(tmp285, tmp289, 221);
		__m512 tmp305 = _mm512_shuffle_f32x4(tmp286, tmp290, 136);
		__m512 tmp306 = _mm512_shuffle_f32x4(tmp286, tmp290, 221);
		in52 = _mm512_shuffle_f32x4(tmp291, tmp299, 136);
		in59 = _mm512_shuffle_f32x4(tmp291, tmp299, 221);
		tmp245 = _mm512_shuffle_f32x4(tmp293, tmp301, 136);
		tmp249 = _mm512_shuffle_f32x4(tmp293, tmp301, 221);
		tmp246 = _mm512_shuffle_f32x4(tmp295, tmp303, 136);
		tmp250 = _mm512_shuffle_f32x4(tmp295, tmp303, 221);
		in58 = _mm512_shuffle_f32x4(tmp297, tmp305, 136);
		in65 = _mm512_shuffle_f32x4(tmp297, tmp305, 221);
		tmp244 = _mm512_shuffle_f32x4(tmp292, tmp300, 136);
		tmp248 = _mm512_shuffle_f32x4(tmp292, tmp300, 221);
		in54 = _mm512_shuffle_f32x4(tmp294, tmp302, 136);
		in61 = _mm512_shuffle_f32x4(tmp294, tmp302, 221);
		in56 = _mm512_shuffle_f32x4(tmp296, tmp304, 136);
		in63 = _mm512_shuffle_f32x4(tmp296, tmp304, 221);
		in55 = _mm512_shuffle_f32x4(tmp298, tmp306, 136);
		in62 = _mm512_shuffle_f32x4(tmp298, tmp306, 221);
		__m512 tmp251 = _mm512_add_ps(tmp245, in54);
		__m512 tmp255 = _mm512_add_ps(tmp249, in61);
		__m512 tmp252 = _mm512_sub_ps(tmp244, tmp246);
		__m512 tmp256 = _mm512_sub_ps(tmp248, tmp250);
		__m512 tmp253 = _mm512_add_ps(tmp246, in56);
		__m512 tmp257 = _mm512_add_ps(tmp250, in63);
		in52 = _mm512_sub_ps(in52, in56);
		in59 = _mm512_sub_ps(in59, in63);
		tmp251 = _mm512_fmadd_ps(in58, _mm512_set1_ps(-4.25e+00f), tmp251);
		tmp255 = _mm512_fmadd_ps(in65, _mm512_set1_ps(-4.25e+00f), tmp255);
		tmp253 = _mm512_fmadd_ps(tmp244, _mm512_set1_ps(-4.25e+00f), tmp253);
		tmp257 = _mm512_fmadd_ps(tmp248, _mm512_set1_ps(-4.25e+00f), tmp257);
		in52 = _mm512_fmadd_ps(tmp252, _mm512_set1_ps(5.25e+00f), in52);
		in59 = _mm512_fmadd_ps(tmp256, _mm512_set1_ps(5.25e+00f), in59);
		tmp252 = _mm512_fmadd_ps(tmp246, _mm512_set1_ps(2.5e-01f), in56);
		tmp256 = _mm512_fmadd_ps(tmp250, _mm512_set1_ps(2.5e-01f), in63);
		tmp246 = _mm512_fmadd_ps(tmp246, _mm512_set1_ps(4e+00f), in56);
		tmp250 = _mm512_fmadd_ps(tmp250, _mm512_set1_ps(4e+00f), in63);
		__m512 tmp254 = _mm512_sub_ps(tmp253, tmp251);
		__m512 tmp258 = _mm512_sub_ps(tmp257, tmp255);
		tmp253 = _mm512_add_ps(tmp251, tmp253);
		tmp257 = _mm512_add_ps(tmp255, tmp257);
		tmp251 = _mm512_fmadd_ps(tmp245, _mm512_set1_ps(2.5e-01f), in54);
		tmp255 = _mm512_fmadd_ps(tmp249, _mm512_set1_ps(2.5e-01f), in61);
		tmp252 = _mm512_fmadd_ps(tmp244, _mm512_set1_ps(-1.25e+00f), tmp252);
		tmp256 = _mm512_fmadd_ps(tmp248, _mm512_set1_ps(-1.25e+00f), tmp256);
		tmp244 = _mm512_fmadd_ps(tmp244, _mm512_set1_ps(-5e+00f), tmp246);
		tmp248 = _mm512_fmadd_ps(tmp248, _mm512_set1_ps(-5e+00f), tmp250);
		tmp251 = _mm512_fmadd_ps(in58, _mm512_set1_ps(-1.25e+00f), tmp251);
		tmp255 = _mm512_fmadd_ps(in65, _mm512_set1_ps(-1.25e+00f), tmp255);
		in56 = _mm512_fmadd_ps(tmp251, _mm512_set1_ps(2e+00f), tmp252);
		in63 = _mm512_fmadd_ps(tmp255, _mm512_set1_ps(2e+00f), tmp256);
		tmp252 = _mm512_fnmadd_ps(tmp251, _mm512_set1_ps(2e+00f), tmp252);
		tmp256 = _mm512_fnmadd_ps(tmp255, _mm512_set1_ps(2e+00f), tmp256);
		tmp251 = _mm512_fmadd_ps(in54, _mm512_set1_ps(2.5e-01f), tmp245);
		tmp255 = _mm512_fmadd_ps(in61, _mm512_set1_ps(2.5e-01f), tmp249);
		tmp245 = _mm512_sub_ps(in55, tmp245);
		tmp249 = _mm512_sub_ps(in62, tmp249);
		tmp251 = _mm512_fmadd_ps(in58, _mm512_set1_ps(-1.25e+00f), tmp251);
		tmp255 = _mm512_fmadd_ps(in65, _mm512_set1_ps(-1.25e+00f), tmp255);
		in58 = _mm512_sub_ps(in58, in54);
		in65 = _mm512_sub_ps(in65, in61);
		in58 = _mm512_fmadd_ps(in58, _mm512_set1_ps(5.25e+00f), tmp245);
		in65 = _mm512_fmadd_ps(in65, _mm512_set1_ps(5.25e+00f), tmp249);
		tmp246 = _mm512_fmadd_ps(tmp251, _mm512_set1_ps(2e+00f), tmp244);
		tmp250 = _mm512_fmadd_ps(tmp255, _mm512_set1_ps(2e+00f), tmp248);
		tmp244 = _mm512_fnmadd_ps(tmp251, _mm512_set1_ps(2e+00f), tmp244);
		tmp248 = _mm512_fnmadd_ps(tmp255, _mm512_set1_ps(2e+00f), tmp248);
		__m512 out65 = _mm512_shuffle_f32x4(in52, tmp253, 68);
		__m512 out73 = _mm512_shuffle_f32x4(in52, tmp253, 238);
		__m512 out66 = _mm512_shuffle_f32x4(tmp254, in56, 68);
		__m512 out74 = _mm512_shuffle_f32x4(tmp254, in56, 238);
		__m512 out67 = _mm512_shuffle_f32x4(tmp252, tmp246, 68);
		__m512 out75 = _mm512_shuffle_f32x4(tmp252, tmp246, 238);
		__m512 out68 = _mm512_shuffle_f32x4(tmp244, in58, 68);
		__m512 out76 = _mm512_shuffle_f32x4(tmp244, in58, 238);
		__m512 out69 = _mm512_shuffle_f32x4(in59, tmp257, 68);
		__m512 out77 = _mm512_shuffle_f32x4(in59, tmp257, 238);
		__m512 out70 = _mm512_shuffle_f32x4(tmp258, in63, 68);
		__m512 out78 = _mm512_shuffle_f32x4(tmp258, in63, 238);
		__m512 out71 = _mm512_shuffle_f32x4(tmp256, tmp250, 68);
		__m512 out79 = _mm512_shuffle_f32x4(tmp256, tmp250, 238);
		__m512 out72 = _mm512_shuffle_f32x4(tmp248, in65, 68);
		__m512 out80 = _mm512_shuffle_f32x4(tmp248, in65, 238);
		_mm512_storeu_ps(dfPtr1+0+107520*i6+16128*j2+10752*s2+256*k3, out65);
		_mm512_storeu_ps(dfPtr1+128+107520*i6+16128*j2+10752*s2+256*k3, out73);
		_mm512_storeu_ps(dfPtr1+64+107520*i6+16128*j2+10752*s2+256*k3, out69);
		_mm512_storeu_ps(dfPtr1+192+107520*i6+16128*j2+10752*s2+256*k3, out77);
		_mm512_storeu_ps(dfPtr1+26880+107520*i6+16128*j2+10752*s2+256*k3, out66);
		_mm512_storeu_ps(dfPtr1+27008+107520*i6+16128*j2+10752*s2+256*k3, out74);
		_mm512_storeu_ps(dfPtr1+26944+107520*i6+16128*j2+10752*s2+256*k3, out70);
		_mm512_storeu_ps(dfPtr1+27072+107520*i6+16128*j2+10752*s2+256*k3, out78);
		_mm512_storeu_ps(dfPtr1+53760+107520*i6+16128*j2+10752*s2+256*k3, out67);
		_mm512_storeu_ps(dfPtr1+53888+107520*i6+16128*j2+10752*s2+256*k3, out75);
		_mm512_storeu_ps(dfPtr1+53824+107520*i6+16128*j2+10752*s2+256*k3, out71);
		_mm512_storeu_ps(dfPtr1+53952+107520*i6+16128*j2+10752*s2+256*k3, out79);
		_mm512_storeu_ps(dfPtr1+80640+107520*i6+16128*j2+10752*s2+256*k3, out68);
		_mm512_storeu_ps(dfPtr1+80768+107520*i6+16128*j2+10752*s2+256*k3, out76);
		_mm512_storeu_ps(dfPtr1+80704+107520*i6+16128*j2+10752*s2+256*k3, out72);
		_mm512_storeu_ps(dfPtr1+80832+107520*i6+16128*j2+10752*s2+256*k3, out80);
	}
	++j2;
}

static void Example28ThreeArrangeDats1(Example28ThreaderTeam1* team15, char** tensors3) {
	Example28ThreaderTask1 task7;
	task7.callee1 = Example28ThreeArrangeDats1Callee1;
	task7.any1 = tensors3;
	task7.nd1 = 4;
	task7.hull1[0] = 1;
	task7.hull1[1] = 1;
	task7.hull1[2] = 1;
	task7.hull1[3] = 1;
	Example28ThreaderDo1(team15, &task7);
}

static void Example28ThreeProduceSums1Callee1(Example28ThreaderTask1* task8, int64_t* pt9) {
	void** pair2 = task8->any1;
	char** tensors6 = pair2[0];
	ptrdiff_t e3 = 0;
	ptrdiff_t g4 = 0;
	ptrdiff_t f2 = pt9[2];
	ptrdiff_t d1 = pt9[1];
	ptrdiff_t w3 = 0;
	char*restrict bfPtr2 = tensors6[0]+272*e3;
	char*restrict wfPtr2 = tensors6[0]+320+3446784*e3;
	char*restrict dfPtr2 = tensors6[1]+1013760*e3;
	char*restrict sfPtr1 = tensors6[2];
	ptrdiff_t i7 = 1*g4;
	ptrdiff_t j3 = 1*f2;
	ptrdiff_t k4 = 1*d1;
	ptrdiff_t kk1 = k4+0;
	for (; k4 != 1; ++k4) {
		ptrdiff_t l1 = 17*w3;
		for (; l1 != 17; ++l1) {
			__m512 sum2;
			__m512 sum8;
			__m512 sum14;
			__m512 sum20;
			if (__builtin_expect(!j3, 0)) {
				sum2 = _mm512_mask_mov_ps(_mm512_setzero_ps(), 512, _mm512_set1_ps(*(float*)(bfPtr2+0+272*i7+16*l1)));
				sum8 = _mm512_mask_mov_ps(_mm512_setzero_ps(), 512, _mm512_set1_ps(*(float*)(bfPtr2+4+272*i7+16*l1)));
				sum14 = _mm512_mask_mov_ps(_mm512_setzero_ps(), 512, _mm512_set1_ps(*(float*)(bfPtr2+8+272*i7+16*l1)));
				sum20 = _mm512_mask_mov_ps(_mm512_setzero_ps(), 512, _mm512_set1_ps(*(float*)(bfPtr2+12+272*i7+16*l1)));
			} else {
				sum2 = _mm512_setzero_ps();
				sum8 = _mm512_setzero_ps();
				sum14 = _mm512_setzero_ps();
				sum20 = _mm512_setzero_ps();
			}
			__m512 sum3 = sum2;
			__m512 sum4 = sum2;
			__m512 sum5 = sum2;
			__m512 sum6 = sum2;
			__m512 sum7 = sum2;
			__m512 sum9 = sum8;
			__m512 sum10 = sum8;
			__m512 sum11 = sum8;
			__m512 sum12 = sum8;
			__m512 sum13 = sum8;
			__m512 sum15 = sum14;
			__m512 sum16 = sum14;
			__m512 sum17 = sum14;
			__m512 sum18 = sum14;
			__m512 sum19 = sum14;
			__m512 sum21 = sum20;
			__m512 sum22 = sum20;
			__m512 sum23 = sum20;
			__m512 sum24 = sum20;
			__m512 sum25 = sum20;
			ptrdiff_t b3 = 0;
			for (; b3 != 42; ++b3) {
				__m512i wfs1 = _mm512_maskz_loadu_epi32(65535, wfPtr2+0+365568*i7+91392*j3+5376*l1+128*b3);
				__m512 wf17 = _mm512_cvtph_ps(_mm512_castsi512_si256(wfs1));
				__m512 df1 = _mm512_loadu_ps(dfPtr2+0+107520*i7+26880*j3+16128*k4+384*b3);
				sum2 = _mm512_fmadd_ps(wf17, df1, sum2);
				__m512 df2 = _mm512_loadu_ps(dfPtr2+64+107520*i7+26880*j3+16128*k4+384*b3);
				sum3 = _mm512_fmadd_ps(wf17, df2, sum3);
				__m512 df3 = _mm512_loadu_ps(dfPtr2+128+107520*i7+26880*j3+16128*k4+384*b3);
				sum4 = _mm512_fmadd_ps(wf17, df3, sum4);
				__m512 df4 = _mm512_loadu_ps(dfPtr2+192+107520*i7+26880*j3+16128*k4+384*b3);
				sum5 = _mm512_fmadd_ps(wf17, df4, sum5);
				__m512 df5 = _mm512_loadu_ps(dfPtr2+256+107520*i7+26880*j3+16128*k4+384*b3);
				sum6 = _mm512_fmadd_ps(wf17, df5, sum6);
				__m512 df6 = _mm512_loadu_ps(dfPtr2+320+107520*i7+26880*j3+16128*k4+384*b3);
				sum7 = _mm512_fmadd_ps(wf17, df6, sum7);
				__m512 wf18 = _mm512_cvtph_ps(_mm512_extracti64x4_epi64(wfs1, 1));
				sum8 = _mm512_fmadd_ps(wf18, df1, sum8);
				sum9 = _mm512_fmadd_ps(wf18, df2, sum9);
				sum10 = _mm512_fmadd_ps(wf18, df3, sum10);
				sum11 = _mm512_fmadd_ps(wf18, df4, sum11);
				sum12 = _mm512_fmadd_ps(wf18, df5, sum12);
				sum13 = _mm512_fmadd_ps(wf18, df6, sum13);
				__m512i wfs2 = _mm512_maskz_loadu_epi32(65535, wfPtr2+64+365568*i7+91392*j3+5376*l1+128*b3);
				__m512 wf19 = _mm512_cvtph_ps(_mm512_castsi512_si256(wfs2));
				sum14 = _mm512_fmadd_ps(wf19, df1, sum14);
				sum15 = _mm512_fmadd_ps(wf19, df2, sum15);
				sum16 = _mm512_fmadd_ps(wf19, df3, sum16);
				sum17 = _mm512_fmadd_ps(wf19, df4, sum17);
				sum18 = _mm512_fmadd_ps(wf19, df5, sum18);
				sum19 = _mm512_fmadd_ps(wf19, df6, sum19);
				__m512 wf20 = _mm512_cvtph_ps(_mm512_extracti64x4_epi64(wfs2, 1));
				sum20 = _mm512_fmadd_ps(wf20, df1, sum20);
				sum21 = _mm512_fmadd_ps(wf20, df2, sum21);
				sum22 = _mm512_fmadd_ps(wf20, df3, sum22);
				sum23 = _mm512_fmadd_ps(wf20, df4, sum23);
				sum24 = _mm512_fmadd_ps(wf20, df5, sum24);
				sum25 = _mm512_fmadd_ps(wf20, df6, sum25);
			}
			_mm512_storeu_ps(sfPtr1+0+174080*i7+43520*j3+26112*k4+1536*l1, sum2);
			_mm512_storeu_ps(sfPtr1+64+174080*i7+43520*j3+26112*k4+1536*l1, sum3);
			_mm512_storeu_ps(sfPtr1+128+174080*i7+43520*j3+26112*k4+1536*l1, sum4);
			_mm512_storeu_ps(sfPtr1+192+174080*i7+43520*j3+26112*k4+1536*l1, sum5);
			_mm512_storeu_ps(sfPtr1+256+174080*i7+43520*j3+26112*k4+1536*l1, sum6);
			_mm512_storeu_ps(sfPtr1+320+174080*i7+43520*j3+26112*k4+1536*l1, sum7);
			_mm512_storeu_ps(sfPtr1+384+174080*i7+43520*j3+26112*k4+1536*l1, sum8);
			_mm512_storeu_ps(sfPtr1+448+174080*i7+43520*j3+26112*k4+1536*l1, sum9);
			_mm512_storeu_ps(sfPtr1+512+174080*i7+43520*j3+26112*k4+1536*l1, sum10);
			_mm512_storeu_ps(sfPtr1+576+174080*i7+43520*j3+26112*k4+1536*l1, sum11);
			_mm512_storeu_ps(sfPtr1+640+174080*i7+43520*j3+26112*k4+1536*l1, sum12);
			_mm512_storeu_ps(sfPtr1+704+174080*i7+43520*j3+26112*k4+1536*l1, sum13);
			_mm512_storeu_ps(sfPtr1+768+174080*i7+43520*j3+26112*k4+1536*l1, sum14);
			_mm512_storeu_ps(sfPtr1+832+174080*i7+43520*j3+26112*k4+1536*l1, sum15);
			_mm512_storeu_ps(sfPtr1+896+174080*i7+43520*j3+26112*k4+1536*l1, sum16);
			_mm512_storeu_ps(sfPtr1+960+174080*i7+43520*j3+26112*k4+1536*l1, sum17);
			_mm512_storeu_ps(sfPtr1+1024+174080*i7+43520*j3+26112*k4+1536*l1, sum18);
			_mm512_storeu_ps(sfPtr1+1088+174080*i7+43520*j3+26112*k4+1536*l1, sum19);
			_mm512_storeu_ps(sfPtr1+1152+174080*i7+43520*j3+26112*k4+1536*l1, sum20);
			_mm512_storeu_ps(sfPtr1+1216+174080*i7+43520*j3+26112*k4+1536*l1, sum21);
			_mm512_storeu_ps(sfPtr1+1280+174080*i7+43520*j3+26112*k4+1536*l1, sum22);
			_mm512_storeu_ps(sfPtr1+1344+174080*i7+43520*j3+26112*k4+1536*l1, sum23);
			_mm512_storeu_ps(sfPtr1+1408+174080*i7+43520*j3+26112*k4+1536*l1, sum24);
			_mm512_storeu_ps(sfPtr1+1472+174080*i7+43520*j3+26112*k4+1536*l1, sum25);
		}
		if (k4 >= kk1) return;
	}
	ptrdiff_t l2 = 17*w3;
	for (; l2 != 17; ++l2) {
		__m512 sum26;
		__m512 sum30;
		__m512 sum34;
		__m512 sum38;
		if (__builtin_expect(!j3, 0)) {
			sum26 = _mm512_mask_mov_ps(_mm512_setzero_ps(), 512, _mm512_set1_ps(*(float*)(bfPtr2+0+272*i7+16*l2)));
			sum30 = _mm512_mask_mov_ps(_mm512_setzero_ps(), 512, _mm512_set1_ps(*(float*)(bfPtr2+4+272*i7+16*l2)));
			sum34 = _mm512_mask_mov_ps(_mm512_setzero_ps(), 512, _mm512_set1_ps(*(float*)(bfPtr2+8+272*i7+16*l2)));
			sum38 = _mm512_mask_mov_ps(_mm512_setzero_ps(), 512, _mm512_set1_ps(*(float*)(bfPtr2+12+272*i7+16*l2)));
		} else {
			sum26 = _mm512_setzero_ps();
			sum30 = _mm512_setzero_ps();
			sum34 = _mm512_setzero_ps();
			sum38 = _mm512_setzero_ps();
		}
		__m512 sum27 = sum26;
		__m512 sum28 = sum26;
		__m512 sum29 = sum26;
		__m512 sum31 = sum30;
		__m512 sum32 = sum30;
		__m512 sum33 = sum30;
		__m512 sum35 = sum34;
		__m512 sum36 = sum34;
		__m512 sum37 = sum34;
		__m512 sum39 = sum38;
		__m512 sum40 = sum38;
		__m512 sum41 = sum38;
		ptrdiff_t b4 = 0;
		for (; b4 != 42; ++b4) {
			__m512i wfs3 = _mm512_maskz_loadu_epi32(65535, wfPtr2+0+365568*i7+91392*j3+5376*l2+128*b4);
			__m512 wf21 = _mm512_cvtph_ps(_mm512_castsi512_si256(wfs3));
			__m512 df7 = _mm512_loadu_ps(dfPtr2+0+107520*i7+26880*j3+16128*k4+256*b4);
			sum26 = _mm512_fmadd_ps(wf21, df7, sum26);
			__m512 df8 = _mm512_loadu_ps(dfPtr2+64+107520*i7+26880*j3+16128*k4+256*b4);
			sum27 = _mm512_fmadd_ps(wf21, df8, sum27);
			__m512 df9 = _mm512_loadu_ps(dfPtr2+128+107520*i7+26880*j3+16128*k4+256*b4);
			sum28 = _mm512_fmadd_ps(wf21, df9, sum28);
			__m512 df10 = _mm512_loadu_ps(dfPtr2+192+107520*i7+26880*j3+16128*k4+256*b4);
			sum29 = _mm512_fmadd_ps(wf21, df10, sum29);
			__m512 wf22 = _mm512_cvtph_ps(_mm512_extracti64x4_epi64(wfs3, 1));
			sum30 = _mm512_fmadd_ps(wf22, df7, sum30);
			sum31 = _mm512_fmadd_ps(wf22, df8, sum31);
			sum32 = _mm512_fmadd_ps(wf22, df9, sum32);
			sum33 = _mm512_fmadd_ps(wf22, df10, sum33);
			__m512i wfs4 = _mm512_maskz_loadu_epi32(65535, wfPtr2+64+365568*i7+91392*j3+5376*l2+128*b4);
			__m512 wf23 = _mm512_cvtph_ps(_mm512_castsi512_si256(wfs4));
			sum34 = _mm512_fmadd_ps(wf23, df7, sum34);
			sum35 = _mm512_fmadd_ps(wf23, df8, sum35);
			sum36 = _mm512_fmadd_ps(wf23, df9, sum36);
			sum37 = _mm512_fmadd_ps(wf23, df10, sum37);
			__m512 wf24 = _mm512_cvtph_ps(_mm512_extracti64x4_epi64(wfs4, 1));
			sum38 = _mm512_fmadd_ps(wf24, df7, sum38);
			sum39 = _mm512_fmadd_ps(wf24, df8, sum39);
			sum40 = _mm512_fmadd_ps(wf24, df9, sum40);
			sum41 = _mm512_fmadd_ps(wf24, df10, sum41);
		}
		_mm512_storeu_ps(sfPtr1+0+174080*i7+43520*j3+26112*k4+1024*l2, sum26);
		_mm512_storeu_ps(sfPtr1+64+174080*i7+43520*j3+26112*k4+1024*l2, sum27);
		_mm512_storeu_ps(sfPtr1+128+174080*i7+43520*j3+26112*k4+1024*l2, sum28);
		_mm512_storeu_ps(sfPtr1+192+174080*i7+43520*j3+26112*k4+1024*l2, sum29);
		_mm512_storeu_ps(sfPtr1+256+174080*i7+43520*j3+26112*k4+1024*l2, sum30);
		_mm512_storeu_ps(sfPtr1+320+174080*i7+43520*j3+26112*k4+1024*l2, sum31);
		_mm512_storeu_ps(sfPtr1+384+174080*i7+43520*j3+26112*k4+1024*l2, sum32);
		_mm512_storeu_ps(sfPtr1+448+174080*i7+43520*j3+26112*k4+1024*l2, sum33);
		_mm512_storeu_ps(sfPtr1+512+174080*i7+43520*j3+26112*k4+1024*l2, sum34);
		_mm512_storeu_ps(sfPtr1+576+174080*i7+43520*j3+26112*k4+1024*l2, sum35);
		_mm512_storeu_ps(sfPtr1+640+174080*i7+43520*j3+26112*k4+1024*l2, sum36);
		_mm512_storeu_ps(sfPtr1+704+174080*i7+43520*j3+26112*k4+1024*l2, sum37);
		_mm512_storeu_ps(sfPtr1+768+174080*i7+43520*j3+26112*k4+1024*l2, sum38);
		_mm512_storeu_ps(sfPtr1+832+174080*i7+43520*j3+26112*k4+1024*l2, sum39);
		_mm512_storeu_ps(sfPtr1+896+174080*i7+43520*j3+26112*k4+1024*l2, sum40);
		_mm512_storeu_ps(sfPtr1+960+174080*i7+43520*j3+26112*k4+1024*l2, sum41);
	}
}

static void Example28ThreeProduceSums1(Example28ThreaderTeam1* team16, char** tensors5) {
	void* pair1[] = {tensors5, 0};
	Example28ThreaderTask1 task9;
	task9.callee1 = Example28ThreeProduceSums1Callee1;
	task9.any1 = pair1;
	task9.nd1 = 4;
	task9.hull1[0] = 1;
	task9.hull1[1] = 2;
	task9.hull1[2] = 4;
	task9.hull1[3] = 1;
	Example28ThreaderDo1(team16, &task9);
}

static void Example28ThreeConsumeSums1Callee1(Example28ThreaderTask1* task10, int64_t* pt10) {
	char** tensors8 = task10->any1;
	ptrdiff_t w4 = 0;
	ptrdiff_t d2 = 0;
	ptrdiff_t g5 = 0;
	(void)pt10;
	char*restrict sfPtr2 = tensors8[0];
	char*restrict datPtr3 = tensors8[1];
	char*restrict bnPtr4 = tensors8[2];
	char*restrict datPtr4 = tensors8[3];
	ptrdiff_t i8 = 1*g5;
	ptrdiff_t j4 = 2*d2;
	ptrdiff_t rel2 = j4-0;
	ptrdiff_t base2 = 0;
	if (rel2 < 1) {
		ptrdiff_t toH1 = base2+0;
		ptrdiff_t toW1 = 0;
		ptrdiff_t k5 = 17*w4;
		for (; k5 != 17; ++k5) {
			ptrdiff_t l3 = 0;
			for (; l3 != 2; ++l3) {
				__m512 sf1 = _mm512_loadu_ps(sfPtr2+0+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf2 = _mm512_loadu_ps(sfPtr2+128+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in66 = _mm512_shuffle_f32x4(sf1, sf2, 68);
				__m512 in67 = _mm512_shuffle_f32x4(sf1, sf2, 238);
				__m512 sf3 = _mm512_loadu_ps(sfPtr2+64+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf4 = _mm512_loadu_ps(sfPtr2+192+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in74 = _mm512_shuffle_f32x4(sf3, sf4, 68);
				__m512 in75 = _mm512_shuffle_f32x4(sf3, sf4, 238);
				__m512 sf5 = _mm512_loadu_ps(sfPtr2+43520+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf6 = _mm512_loadu_ps(sfPtr2+43648+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in68 = _mm512_shuffle_f32x4(sf5, sf6, 68);
				__m512 in69 = _mm512_shuffle_f32x4(sf5, sf6, 238);
				__m512 sf7 = _mm512_loadu_ps(sfPtr2+43584+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf8 = _mm512_loadu_ps(sfPtr2+43712+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in76 = _mm512_shuffle_f32x4(sf7, sf8, 68);
				__m512 in77 = _mm512_shuffle_f32x4(sf7, sf8, 238);
				__m512 sf9 = _mm512_loadu_ps(sfPtr2+87040+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf10 = _mm512_loadu_ps(sfPtr2+87168+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in70 = _mm512_shuffle_f32x4(sf9, sf10, 68);
				__m512 in71 = _mm512_shuffle_f32x4(sf9, sf10, 238);
				__m512 sf11 = _mm512_loadu_ps(sfPtr2+87104+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf12 = _mm512_loadu_ps(sfPtr2+87232+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in78 = _mm512_shuffle_f32x4(sf11, sf12, 68);
				__m512 in79 = _mm512_shuffle_f32x4(sf11, sf12, 238);
				__m512 sf13 = _mm512_loadu_ps(sfPtr2+130560+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf14 = _mm512_loadu_ps(sfPtr2+130688+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in72 = _mm512_shuffle_f32x4(sf13, sf14, 68);
				__m512 in73 = _mm512_shuffle_f32x4(sf13, sf14, 238);
				__m512 sf15 = _mm512_loadu_ps(sfPtr2+130624+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf16 = _mm512_loadu_ps(sfPtr2+130752+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in80 = _mm512_shuffle_f32x4(sf15, sf16, 68);
				__m512 in81 = _mm512_shuffle_f32x4(sf15, sf16, 238);
				__m512 tmp323 = _mm512_add_ps(in67, in68);
				__m512 tmp343 = _mm512_add_ps(in75, in76);
				__m512 tmp322 = _mm512_add_ps(in69, in70);
				__m512 tmp342 = _mm512_add_ps(in77, in78);
				__m512 tmp328 = _mm512_sub_ps(in69, in70);
				__m512 tmp348 = _mm512_sub_ps(in77, in78);
				__m512 tmp327 = _mm512_sub_ps(in67, in68);
				__m512 tmp347 = _mm512_sub_ps(in75, in76);
				__m512 tmp324 = _mm512_add_ps(in71, in72);
				__m512 tmp344 = _mm512_add_ps(in79, in80);
				__m512 tmp329 = _mm512_sub_ps(in71, in72);
				__m512 tmp349 = _mm512_sub_ps(in79, in80);
				__m512 tmp326 = _mm512_fmadd_ps(tmp328, _mm512_set1_ps(2e+00f), tmp327);
				__m512 tmp346 = _mm512_fmadd_ps(tmp348, _mm512_set1_ps(2e+00f), tmp347);
				__m512 tmp333 = _mm512_fmadd_ps(tmp328, _mm512_set1_ps(8e+00f), tmp327);
				__m512 tmp353 = _mm512_fmadd_ps(tmp348, _mm512_set1_ps(8e+00f), tmp347);
				__m512 tmp321 = _mm512_add_ps(tmp322, tmp323);
				__m512 tmp341 = _mm512_add_ps(tmp342, tmp343);
				__m512 tmp325 = _mm512_fmadd_ps(tmp329, _mm512_set1_ps(1.6e+01f), tmp326);
				__m512 tmp345 = _mm512_fmadd_ps(tmp349, _mm512_set1_ps(1.6e+01f), tmp346);
				__m512 tmp332 = _mm512_fmadd_ps(tmp329, _mm512_set1_ps(4e+00f), tmp333);
				__m512 tmp352 = _mm512_fmadd_ps(tmp349, _mm512_set1_ps(4e+00f), tmp353);
				__m512 tmp338 = _mm512_add_ps(tmp329, tmp327);
				__m512 tmp358 = _mm512_add_ps(tmp349, tmp347);
				__m512 tmp331 = _mm512_fmadd_ps(tmp322, _mm512_set1_ps(4e+00f), tmp323);
				__m512 tmp351 = _mm512_fmadd_ps(tmp342, _mm512_set1_ps(4e+00f), tmp343);
				__m512 tmp335 = _mm512_fmadd_ps(tmp322, _mm512_set1_ps(1.6e+01f), tmp323);
				__m512 tmp355 = _mm512_fmadd_ps(tmp342, _mm512_set1_ps(1.6e+01f), tmp343);
				__m512 tmp320 = _mm512_add_ps(tmp321, in66);
				__m512 tmp340 = _mm512_add_ps(tmp341, in74);
				__m512 tmp337 = _mm512_add_ps(tmp338, in73);
				__m512 tmp357 = _mm512_add_ps(tmp358, in81);
				__m512 tmp319 = _mm512_fmadd_ps(tmp324, _mm512_set1_ps(3.2e+01f), tmp320);
				__m512 tmp339 = _mm512_fmadd_ps(tmp344, _mm512_set1_ps(3.2e+01f), tmp340);
				__m512 tmp330 = _mm512_fmadd_ps(tmp324, _mm512_set1_ps(8e+00f), tmp331);
				__m512 tmp350 = _mm512_fmadd_ps(tmp344, _mm512_set1_ps(8e+00f), tmp351);
				__m512 tmp336 = _mm512_fmadd_ps(tmp328, _mm512_set1_ps(3.2e+01f), tmp337);
				__m512 tmp356 = _mm512_fmadd_ps(tmp348, _mm512_set1_ps(3.2e+01f), tmp357);
				__m512 tmp334 = _mm512_fmadd_ps(tmp324, _mm512_set1_ps(2e+00f), tmp335);
				__m512 tmp354 = _mm512_fmadd_ps(tmp344, _mm512_set1_ps(2e+00f), tmp355);
				__m512 tmp307 = tmp319;
				__m512 tmp313 = tmp339;
				__m512 tmp308 = tmp325;
				__m512 tmp314 = tmp345;
				__m512 tmp309 = tmp330;
				__m512 tmp315 = tmp350;
				__m512 tmp310 = tmp332;
				__m512 tmp316 = tmp352;
				__m512 tmp311 = tmp334;
				__m512 tmp317 = tmp354;
				__m512 tmp312 = tmp336;
				__m512 tmp318 = tmp356;
				__m512 tmp403 = _mm512_unpacklo_ps(tmp307, tmp308);
				__m512 tmp404 = _mm512_unpackhi_ps(tmp307, tmp308);
				__m512 tmp405 = _mm512_unpacklo_ps(tmp309, tmp310);
				__m512 tmp406 = _mm512_unpackhi_ps(tmp309, tmp310);
				__m512 tmp407 = _mm512_unpacklo_ps(tmp311, tmp312);
				__m512 tmp408 = _mm512_unpackhi_ps(tmp311, tmp312);
				__m512 tmp409 = _mm512_unpacklo_ps(tmp313, tmp314);
				__m512 tmp410 = _mm512_unpackhi_ps(tmp313, tmp314);
				__m512 tmp411 = _mm512_unpacklo_ps(tmp315, tmp316);
				__m512 tmp412 = _mm512_unpackhi_ps(tmp315, tmp316);
				__m512 tmp413 = _mm512_unpacklo_ps(tmp317, tmp318);
				__m512 tmp414 = _mm512_unpackhi_ps(tmp317, tmp318);
				__m512 tmp415 = _mm512_shuffle_ps(tmp403, tmp405, 68);
				__m512 tmp416 = _mm512_shuffle_ps(tmp403, tmp405, 238);
				__m512 tmp417 = _mm512_shuffle_ps(tmp404, tmp406, 68);
				__m512 tmp418 = _mm512_shuffle_ps(tmp404, tmp406, 238);
				__m512 tmp419 = _mm512_shuffle_ps(tmp407, tmp409, 68);
				__m512 tmp420 = _mm512_shuffle_ps(tmp407, tmp409, 238);
				__m512 tmp421 = _mm512_shuffle_ps(tmp408, tmp410, 68);
				__m512 tmp422 = _mm512_shuffle_ps(tmp408, tmp410, 238);
				__m512 tmp423 = _mm512_shuffle_ps(tmp411, tmp413, 68);
				__m512 tmp424 = _mm512_shuffle_ps(tmp411, tmp413, 238);
				__m512 tmp425 = _mm512_shuffle_ps(tmp412, tmp414, 68);
				__m512 tmp426 = _mm512_shuffle_ps(tmp412, tmp414, 238);
				__m512 tmp427 = _mm512_shuffle_f32x4(tmp415, tmp419, 136);
				__m512 tmp428 = _mm512_shuffle_f32x4(tmp415, tmp419, 221);
				__m512 tmp429 = _mm512_shuffle_f32x4(tmp416, tmp420, 136);
				__m512 tmp430 = _mm512_shuffle_f32x4(tmp416, tmp420, 221);
				__m512 tmp431 = _mm512_shuffle_f32x4(tmp417, tmp421, 136);
				__m512 tmp432 = _mm512_shuffle_f32x4(tmp417, tmp421, 221);
				__m512 tmp433 = _mm512_shuffle_f32x4(tmp418, tmp422, 136);
				__m512 tmp434 = _mm512_shuffle_f32x4(tmp418, tmp422, 221);
				__m512 tmp435 = _mm512_shuffle_f32x4(tmp423, tmp423, 136);
				__m512 tmp436 = _mm512_shuffle_f32x4(tmp423, tmp423, 221);
				__m512 tmp437 = _mm512_shuffle_f32x4(tmp424, tmp424, 136);
				__m512 tmp438 = _mm512_shuffle_f32x4(tmp424, tmp424, 221);
				__m512 tmp439 = _mm512_shuffle_f32x4(tmp425, tmp425, 136);
				__m512 tmp440 = _mm512_shuffle_f32x4(tmp425, tmp425, 221);
				__m512 tmp441 = _mm512_shuffle_f32x4(tmp426, tmp426, 136);
				__m512 tmp442 = _mm512_shuffle_f32x4(tmp426, tmp426, 221);
				tmp307 = _mm512_shuffle_f32x4(tmp427, tmp435, 136);
				tmp315 = _mm512_shuffle_f32x4(tmp427, tmp435, 221);
				tmp308 = _mm512_shuffle_f32x4(tmp429, tmp437, 136);
				tmp316 = _mm512_shuffle_f32x4(tmp429, tmp437, 221);
				tmp309 = _mm512_shuffle_f32x4(tmp431, tmp439, 136);
				tmp317 = _mm512_shuffle_f32x4(tmp431, tmp439, 221);
				tmp310 = _mm512_shuffle_f32x4(tmp433, tmp441, 136);
				tmp318 = _mm512_shuffle_f32x4(tmp433, tmp441, 221);
				tmp311 = _mm512_shuffle_f32x4(tmp428, tmp436, 136);
				__m512 tmp359 = _mm512_shuffle_f32x4(tmp428, tmp436, 221);
				tmp312 = _mm512_shuffle_f32x4(tmp430, tmp438, 136);
				__m512 tmp360 = _mm512_shuffle_f32x4(tmp430, tmp438, 221);
				tmp313 = _mm512_shuffle_f32x4(tmp432, tmp440, 136);
				__m512 tmp361 = _mm512_shuffle_f32x4(tmp432, tmp440, 221);
				tmp314 = _mm512_shuffle_f32x4(tmp434, tmp442, 136);
				__m512 tmp362 = _mm512_shuffle_f32x4(tmp434, tmp442, 221);
				__m512 tmp367 = _mm512_add_ps(tmp308, tmp309);
				__m512 tmp387 = _mm512_add_ps(tmp316, tmp317);
				__m512 tmp366 = _mm512_add_ps(tmp310, tmp311);
				__m512 tmp386 = _mm512_add_ps(tmp318, tmp359);
				__m512 tmp372 = _mm512_sub_ps(tmp310, tmp311);
				__m512 tmp392 = _mm512_sub_ps(tmp318, tmp359);
				__m512 tmp371 = _mm512_sub_ps(tmp308, tmp309);
				__m512 tmp391 = _mm512_sub_ps(tmp316, tmp317);
				__m512 tmp368 = _mm512_add_ps(tmp312, tmp313);
				__m512 tmp388 = _mm512_add_ps(tmp360, tmp361);
				__m512 tmp373 = _mm512_sub_ps(tmp312, tmp313);
				__m512 tmp393 = _mm512_sub_ps(tmp360, tmp361);
				__m512 tmp370 = _mm512_fmadd_ps(tmp372, _mm512_set1_ps(2e+00f), tmp371);
				__m512 tmp390 = _mm512_fmadd_ps(tmp392, _mm512_set1_ps(2e+00f), tmp391);
				__m512 tmp377 = _mm512_fmadd_ps(tmp372, _mm512_set1_ps(8e+00f), tmp371);
				__m512 tmp397 = _mm512_fmadd_ps(tmp392, _mm512_set1_ps(8e+00f), tmp391);
				__m512 tmp365 = _mm512_add_ps(tmp366, tmp367);
				__m512 tmp385 = _mm512_add_ps(tmp386, tmp387);
				__m512 tmp369 = _mm512_fmadd_ps(tmp373, _mm512_set1_ps(1.6e+01f), tmp370);
				__m512 tmp389 = _mm512_fmadd_ps(tmp393, _mm512_set1_ps(1.6e+01f), tmp390);
				__m512 tmp376 = _mm512_fmadd_ps(tmp373, _mm512_set1_ps(4e+00f), tmp377);
				__m512 tmp396 = _mm512_fmadd_ps(tmp393, _mm512_set1_ps(4e+00f), tmp397);
				__m512 tmp382 = _mm512_add_ps(tmp373, tmp371);
				__m512 tmp402 = _mm512_add_ps(tmp393, tmp391);
				__m512 tmp375 = _mm512_fmadd_ps(tmp366, _mm512_set1_ps(4e+00f), tmp367);
				__m512 tmp395 = _mm512_fmadd_ps(tmp386, _mm512_set1_ps(4e+00f), tmp387);
				__m512 tmp379 = _mm512_fmadd_ps(tmp366, _mm512_set1_ps(1.6e+01f), tmp367);
				__m512 tmp399 = _mm512_fmadd_ps(tmp386, _mm512_set1_ps(1.6e+01f), tmp387);
				__m512 tmp364 = _mm512_add_ps(tmp365, tmp307);
				__m512 tmp384 = _mm512_add_ps(tmp385, tmp315);
				__m512 tmp381 = _mm512_add_ps(tmp382, tmp314);
				__m512 tmp401 = _mm512_add_ps(tmp402, tmp362);
				__m512 tmp363 = _mm512_fmadd_ps(tmp368, _mm512_set1_ps(3.2e+01f), tmp364);
				__m512 tmp383 = _mm512_fmadd_ps(tmp388, _mm512_set1_ps(3.2e+01f), tmp384);
				__m512 tmp374 = _mm512_fmadd_ps(tmp368, _mm512_set1_ps(8e+00f), tmp375);
				__m512 tmp394 = _mm512_fmadd_ps(tmp388, _mm512_set1_ps(8e+00f), tmp395);
				__m512 tmp380 = _mm512_fmadd_ps(tmp372, _mm512_set1_ps(3.2e+01f), tmp381);
				__m512 tmp400 = _mm512_fmadd_ps(tmp392, _mm512_set1_ps(3.2e+01f), tmp401);
				__m512 tmp378 = _mm512_fmadd_ps(tmp368, _mm512_set1_ps(2e+00f), tmp379);
				__m512 tmp398 = _mm512_fmadd_ps(tmp388, _mm512_set1_ps(2e+00f), tmp399);
				__m512 out81 = tmp363;
				__m512 out87 = tmp383;
				__m512 out82 = tmp369;
				__m512 out88 = tmp389;
				__m512 out83 = tmp374;
				__m512 out89 = tmp394;
				__m512 out84 = tmp376;
				__m512 out90 = tmp396;
				__m512 out85 = tmp378;
				__m512 out91 = tmp398;
				__m512 out86 = tmp380;
				__m512 out92 = tmp400;
				out81 = _mm512_max_ps(_mm512_setzero_ps(), out81);
				out87 = _mm512_max_ps(_mm512_setzero_ps(), out87);
				out82 = _mm512_max_ps(_mm512_setzero_ps(), out82);
				out88 = _mm512_max_ps(_mm512_setzero_ps(), out88);
				out83 = _mm512_max_ps(_mm512_setzero_ps(), out83);
				out89 = _mm512_max_ps(_mm512_setzero_ps(), out89);
				out84 = _mm512_max_ps(_mm512_setzero_ps(), out84);
				out90 = _mm512_max_ps(_mm512_setzero_ps(), out90);
				out85 = _mm512_max_ps(_mm512_setzero_ps(), out85);
				out91 = _mm512_max_ps(_mm512_setzero_ps(), out91);
				out86 = _mm512_max_ps(_mm512_setzero_ps(), out86);
				out92 = _mm512_max_ps(_mm512_setzero_ps(), out92);
				out81 = _mm512_add_ps(out81, _mm512_maskz_loadu_ps(4095, datPtr3+0+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out87 = _mm512_add_ps(out87, _mm512_maskz_loadu_ps(4095, datPtr3+48+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out82 = _mm512_add_ps(out82, _mm512_maskz_loadu_ps(4095, datPtr3+120+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out88 = _mm512_add_ps(out88, _mm512_maskz_loadu_ps(4095, datPtr3+168+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out83 = _mm512_add_ps(out83, _mm512_maskz_loadu_ps(4095, datPtr3+240+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out89 = _mm512_add_ps(out89, _mm512_maskz_loadu_ps(4095, datPtr3+288+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out84 = _mm512_add_ps(out84, _mm512_maskz_loadu_ps(4095, datPtr3+360+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out90 = _mm512_add_ps(out90, _mm512_maskz_loadu_ps(4095, datPtr3+408+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out85 = _mm512_add_ps(out85, _mm512_maskz_loadu_ps(4095, datPtr3+480+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out91 = _mm512_add_ps(out91, _mm512_maskz_loadu_ps(4095, datPtr3+528+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out86 = _mm512_add_ps(out86, _mm512_maskz_loadu_ps(4095, datPtr3+600+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out92 = _mm512_add_ps(out92, _mm512_maskz_loadu_ps(4095, datPtr3+648+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				__m512 bnMul4 = _mm512_set1_ps(((float*)bnPtr4+(ptrdiff_t)2*(0+68*i8+4*k5+2*l3))[0]);
				__m512 bnAdd4 = _mm512_set1_ps(((float*)bnPtr4+(ptrdiff_t)2*(0+68*i8+4*k5+2*l3))[1]);
				out81 = _mm512_fmadd_ps(out81, bnMul4, bnAdd4);
				out87 = _mm512_fmadd_ps(out87, bnMul4, bnAdd4);
				out82 = _mm512_fmadd_ps(out82, bnMul4, bnAdd4);
				out88 = _mm512_fmadd_ps(out88, bnMul4, bnAdd4);
				out83 = _mm512_fmadd_ps(out83, bnMul4, bnAdd4);
				out89 = _mm512_fmadd_ps(out89, bnMul4, bnAdd4);
				out84 = _mm512_fmadd_ps(out84, bnMul4, bnAdd4);
				out90 = _mm512_fmadd_ps(out90, bnMul4, bnAdd4);
				out85 = _mm512_fmadd_ps(out85, bnMul4, bnAdd4);
				out91 = _mm512_fmadd_ps(out91, bnMul4, bnAdd4);
				out86 = _mm512_fmadd_ps(out86, bnMul4, bnAdd4);
				out92 = _mm512_fmadd_ps(out92, bnMul4, bnAdd4);
				_mm512_mask_storeu_ps(datPtr4+0+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out81);
				_mm512_mask_storeu_ps(datPtr4+48+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out87);
				_mm512_mask_storeu_ps(datPtr4+120+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out82);
				_mm512_mask_storeu_ps(datPtr4+168+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out88);
				_mm512_mask_storeu_ps(datPtr4+240+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out83);
				_mm512_mask_storeu_ps(datPtr4+288+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out89);
				_mm512_mask_storeu_ps(datPtr4+360+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out84);
				_mm512_mask_storeu_ps(datPtr4+408+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out90);
				_mm512_mask_storeu_ps(datPtr4+480+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out85);
				_mm512_mask_storeu_ps(datPtr4+528+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out91);
				_mm512_mask_storeu_ps(datPtr4+600+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out86);
				_mm512_mask_storeu_ps(datPtr4+648+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out92);
				__m512 sf17 = _mm512_loadu_ps(sfPtr2+256+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf18 = _mm512_loadu_ps(sfPtr2+384+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in82 = _mm512_shuffle_f32x4(sf18, sf17, 68);
				__m512 in83 = _mm512_shuffle_f32x4(sf18, sf17, 238);
				__m512 sf19 = _mm512_loadu_ps(sfPtr2+320+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf20 = _mm512_loadu_ps(sfPtr2+448+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in90 = _mm512_shuffle_f32x4(sf20, sf19, 68);
				__m512 in91 = _mm512_shuffle_f32x4(sf20, sf19, 238);
				__m512 sf21 = _mm512_loadu_ps(sfPtr2+43776+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf22 = _mm512_loadu_ps(sfPtr2+43904+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in84 = _mm512_shuffle_f32x4(sf22, sf21, 68);
				__m512 in85 = _mm512_shuffle_f32x4(sf22, sf21, 238);
				__m512 sf23 = _mm512_loadu_ps(sfPtr2+43840+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf24 = _mm512_loadu_ps(sfPtr2+43968+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in92 = _mm512_shuffle_f32x4(sf24, sf23, 68);
				__m512 in93 = _mm512_shuffle_f32x4(sf24, sf23, 238);
				__m512 sf25 = _mm512_loadu_ps(sfPtr2+87296+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf26 = _mm512_loadu_ps(sfPtr2+87424+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in86 = _mm512_shuffle_f32x4(sf26, sf25, 68);
				__m512 in87 = _mm512_shuffle_f32x4(sf26, sf25, 238);
				__m512 sf27 = _mm512_loadu_ps(sfPtr2+87360+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf28 = _mm512_loadu_ps(sfPtr2+87488+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in94 = _mm512_shuffle_f32x4(sf28, sf27, 68);
				__m512 in95 = _mm512_shuffle_f32x4(sf28, sf27, 238);
				__m512 sf29 = _mm512_loadu_ps(sfPtr2+130816+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf30 = _mm512_loadu_ps(sfPtr2+130944+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in88 = _mm512_shuffle_f32x4(sf30, sf29, 68);
				__m512 in89 = _mm512_shuffle_f32x4(sf30, sf29, 238);
				__m512 sf31 = _mm512_loadu_ps(sfPtr2+130880+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf32 = _mm512_loadu_ps(sfPtr2+131008+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in96 = _mm512_shuffle_f32x4(sf32, sf31, 68);
				__m512 in97 = _mm512_shuffle_f32x4(sf32, sf31, 238);
				__m512 tmp459 = _mm512_add_ps(in83, in84);
				__m512 tmp479 = _mm512_add_ps(in91, in92);
				__m512 tmp458 = _mm512_add_ps(in85, in86);
				__m512 tmp478 = _mm512_add_ps(in93, in94);
				__m512 tmp464 = _mm512_sub_ps(in85, in86);
				__m512 tmp484 = _mm512_sub_ps(in93, in94);
				__m512 tmp463 = _mm512_sub_ps(in83, in84);
				__m512 tmp483 = _mm512_sub_ps(in91, in92);
				__m512 tmp460 = _mm512_add_ps(in87, in88);
				__m512 tmp480 = _mm512_add_ps(in95, in96);
				__m512 tmp465 = _mm512_sub_ps(in87, in88);
				__m512 tmp485 = _mm512_sub_ps(in95, in96);
				__m512 tmp462 = _mm512_fmadd_ps(tmp464, _mm512_set1_ps(2e+00f), tmp463);
				__m512 tmp482 = _mm512_fmadd_ps(tmp484, _mm512_set1_ps(2e+00f), tmp483);
				__m512 tmp469 = _mm512_fmadd_ps(tmp464, _mm512_set1_ps(8e+00f), tmp463);
				__m512 tmp489 = _mm512_fmadd_ps(tmp484, _mm512_set1_ps(8e+00f), tmp483);
				__m512 tmp457 = _mm512_add_ps(tmp458, tmp459);
				__m512 tmp477 = _mm512_add_ps(tmp478, tmp479);
				__m512 tmp461 = _mm512_fmadd_ps(tmp465, _mm512_set1_ps(1.6e+01f), tmp462);
				__m512 tmp481 = _mm512_fmadd_ps(tmp485, _mm512_set1_ps(1.6e+01f), tmp482);
				__m512 tmp468 = _mm512_fmadd_ps(tmp465, _mm512_set1_ps(4e+00f), tmp469);
				__m512 tmp488 = _mm512_fmadd_ps(tmp485, _mm512_set1_ps(4e+00f), tmp489);
				__m512 tmp474 = _mm512_add_ps(tmp465, tmp463);
				__m512 tmp494 = _mm512_add_ps(tmp485, tmp483);
				__m512 tmp467 = _mm512_fmadd_ps(tmp458, _mm512_set1_ps(4e+00f), tmp459);
				__m512 tmp487 = _mm512_fmadd_ps(tmp478, _mm512_set1_ps(4e+00f), tmp479);
				__m512 tmp471 = _mm512_fmadd_ps(tmp458, _mm512_set1_ps(1.6e+01f), tmp459);
				__m512 tmp491 = _mm512_fmadd_ps(tmp478, _mm512_set1_ps(1.6e+01f), tmp479);
				__m512 tmp456 = _mm512_add_ps(tmp457, in82);
				__m512 tmp476 = _mm512_add_ps(tmp477, in90);
				__m512 tmp473 = _mm512_add_ps(tmp474, in89);
				__m512 tmp493 = _mm512_add_ps(tmp494, in97);
				__m512 tmp455 = _mm512_fmadd_ps(tmp460, _mm512_set1_ps(3.2e+01f), tmp456);
				__m512 tmp475 = _mm512_fmadd_ps(tmp480, _mm512_set1_ps(3.2e+01f), tmp476);
				__m512 tmp466 = _mm512_fmadd_ps(tmp460, _mm512_set1_ps(8e+00f), tmp467);
				__m512 tmp486 = _mm512_fmadd_ps(tmp480, _mm512_set1_ps(8e+00f), tmp487);
				__m512 tmp472 = _mm512_fmadd_ps(tmp464, _mm512_set1_ps(3.2e+01f), tmp473);
				__m512 tmp492 = _mm512_fmadd_ps(tmp484, _mm512_set1_ps(3.2e+01f), tmp493);
				__m512 tmp470 = _mm512_fmadd_ps(tmp460, _mm512_set1_ps(2e+00f), tmp471);
				__m512 tmp490 = _mm512_fmadd_ps(tmp480, _mm512_set1_ps(2e+00f), tmp491);
				__m512 tmp443 = tmp455;
				__m512 tmp449 = tmp475;
				__m512 tmp444 = tmp461;
				__m512 tmp450 = tmp481;
				__m512 tmp445 = tmp466;
				__m512 tmp451 = tmp486;
				__m512 tmp446 = tmp468;
				__m512 tmp452 = tmp488;
				__m512 tmp447 = tmp470;
				__m512 tmp453 = tmp490;
				__m512 tmp448 = tmp472;
				__m512 tmp454 = tmp492;
				__m512 tmp539 = _mm512_unpacklo_ps(tmp443, tmp444);
				__m512 tmp540 = _mm512_unpackhi_ps(tmp443, tmp444);
				__m512 tmp541 = _mm512_unpacklo_ps(tmp445, tmp446);
				__m512 tmp542 = _mm512_unpackhi_ps(tmp445, tmp446);
				__m512 tmp543 = _mm512_unpacklo_ps(tmp447, tmp448);
				__m512 tmp544 = _mm512_unpackhi_ps(tmp447, tmp448);
				__m512 tmp545 = _mm512_unpacklo_ps(tmp449, tmp450);
				__m512 tmp546 = _mm512_unpackhi_ps(tmp449, tmp450);
				__m512 tmp547 = _mm512_unpacklo_ps(tmp451, tmp452);
				__m512 tmp548 = _mm512_unpackhi_ps(tmp451, tmp452);
				__m512 tmp549 = _mm512_unpacklo_ps(tmp453, tmp454);
				__m512 tmp550 = _mm512_unpackhi_ps(tmp453, tmp454);
				__m512 tmp551 = _mm512_shuffle_ps(tmp539, tmp541, 68);
				__m512 tmp552 = _mm512_shuffle_ps(tmp539, tmp541, 238);
				__m512 tmp553 = _mm512_shuffle_ps(tmp540, tmp542, 68);
				__m512 tmp554 = _mm512_shuffle_ps(tmp540, tmp542, 238);
				__m512 tmp555 = _mm512_shuffle_ps(tmp543, tmp545, 68);
				__m512 tmp556 = _mm512_shuffle_ps(tmp543, tmp545, 238);
				__m512 tmp557 = _mm512_shuffle_ps(tmp544, tmp546, 68);
				__m512 tmp558 = _mm512_shuffle_ps(tmp544, tmp546, 238);
				__m512 tmp559 = _mm512_shuffle_ps(tmp547, tmp549, 68);
				__m512 tmp560 = _mm512_shuffle_ps(tmp547, tmp549, 238);
				__m512 tmp561 = _mm512_shuffle_ps(tmp548, tmp550, 68);
				__m512 tmp562 = _mm512_shuffle_ps(tmp548, tmp550, 238);
				__m512 tmp563 = _mm512_shuffle_f32x4(tmp551, tmp555, 136);
				__m512 tmp564 = _mm512_shuffle_f32x4(tmp551, tmp555, 221);
				__m512 tmp565 = _mm512_shuffle_f32x4(tmp552, tmp556, 136);
				__m512 tmp566 = _mm512_shuffle_f32x4(tmp552, tmp556, 221);
				__m512 tmp567 = _mm512_shuffle_f32x4(tmp553, tmp557, 136);
				__m512 tmp568 = _mm512_shuffle_f32x4(tmp553, tmp557, 221);
				__m512 tmp569 = _mm512_shuffle_f32x4(tmp554, tmp558, 136);
				__m512 tmp570 = _mm512_shuffle_f32x4(tmp554, tmp558, 221);
				__m512 tmp571 = _mm512_shuffle_f32x4(tmp559, tmp559, 136);
				__m512 tmp572 = _mm512_shuffle_f32x4(tmp559, tmp559, 221);
				__m512 tmp573 = _mm512_shuffle_f32x4(tmp560, tmp560, 136);
				__m512 tmp574 = _mm512_shuffle_f32x4(tmp560, tmp560, 221);
				__m512 tmp575 = _mm512_shuffle_f32x4(tmp561, tmp561, 136);
				__m512 tmp576 = _mm512_shuffle_f32x4(tmp561, tmp561, 221);
				__m512 tmp577 = _mm512_shuffle_f32x4(tmp562, tmp562, 136);
				__m512 tmp578 = _mm512_shuffle_f32x4(tmp562, tmp562, 221);
				tmp443 = _mm512_shuffle_f32x4(tmp563, tmp571, 136);
				tmp451 = _mm512_shuffle_f32x4(tmp563, tmp571, 221);
				tmp444 = _mm512_shuffle_f32x4(tmp565, tmp573, 136);
				tmp452 = _mm512_shuffle_f32x4(tmp565, tmp573, 221);
				tmp445 = _mm512_shuffle_f32x4(tmp567, tmp575, 136);
				tmp453 = _mm512_shuffle_f32x4(tmp567, tmp575, 221);
				tmp446 = _mm512_shuffle_f32x4(tmp569, tmp577, 136);
				tmp454 = _mm512_shuffle_f32x4(tmp569, tmp577, 221);
				tmp447 = _mm512_shuffle_f32x4(tmp564, tmp572, 136);
				__m512 tmp495 = _mm512_shuffle_f32x4(tmp564, tmp572, 221);
				tmp448 = _mm512_shuffle_f32x4(tmp566, tmp574, 136);
				__m512 tmp496 = _mm512_shuffle_f32x4(tmp566, tmp574, 221);
				tmp449 = _mm512_shuffle_f32x4(tmp568, tmp576, 136);
				__m512 tmp497 = _mm512_shuffle_f32x4(tmp568, tmp576, 221);
				tmp450 = _mm512_shuffle_f32x4(tmp570, tmp578, 136);
				__m512 tmp498 = _mm512_shuffle_f32x4(tmp570, tmp578, 221);
				__m512 tmp503 = _mm512_add_ps(tmp444, tmp445);
				__m512 tmp523 = _mm512_add_ps(tmp452, tmp453);
				__m512 tmp502 = _mm512_add_ps(tmp446, tmp447);
				__m512 tmp522 = _mm512_add_ps(tmp454, tmp495);
				__m512 tmp508 = _mm512_sub_ps(tmp446, tmp447);
				__m512 tmp528 = _mm512_sub_ps(tmp454, tmp495);
				__m512 tmp507 = _mm512_sub_ps(tmp444, tmp445);
				__m512 tmp527 = _mm512_sub_ps(tmp452, tmp453);
				__m512 tmp504 = _mm512_add_ps(tmp448, tmp449);
				__m512 tmp524 = _mm512_add_ps(tmp496, tmp497);
				__m512 tmp509 = _mm512_sub_ps(tmp448, tmp449);
				__m512 tmp529 = _mm512_sub_ps(tmp496, tmp497);
				__m512 tmp506 = _mm512_fmadd_ps(tmp508, _mm512_set1_ps(2e+00f), tmp507);
				__m512 tmp526 = _mm512_fmadd_ps(tmp528, _mm512_set1_ps(2e+00f), tmp527);
				__m512 tmp513 = _mm512_fmadd_ps(tmp508, _mm512_set1_ps(8e+00f), tmp507);
				__m512 tmp533 = _mm512_fmadd_ps(tmp528, _mm512_set1_ps(8e+00f), tmp527);
				__m512 tmp501 = _mm512_add_ps(tmp502, tmp503);
				__m512 tmp521 = _mm512_add_ps(tmp522, tmp523);
				__m512 tmp505 = _mm512_fmadd_ps(tmp509, _mm512_set1_ps(1.6e+01f), tmp506);
				__m512 tmp525 = _mm512_fmadd_ps(tmp529, _mm512_set1_ps(1.6e+01f), tmp526);
				__m512 tmp512 = _mm512_fmadd_ps(tmp509, _mm512_set1_ps(4e+00f), tmp513);
				__m512 tmp532 = _mm512_fmadd_ps(tmp529, _mm512_set1_ps(4e+00f), tmp533);
				__m512 tmp518 = _mm512_add_ps(tmp509, tmp507);
				__m512 tmp538 = _mm512_add_ps(tmp529, tmp527);
				__m512 tmp511 = _mm512_fmadd_ps(tmp502, _mm512_set1_ps(4e+00f), tmp503);
				__m512 tmp531 = _mm512_fmadd_ps(tmp522, _mm512_set1_ps(4e+00f), tmp523);
				__m512 tmp515 = _mm512_fmadd_ps(tmp502, _mm512_set1_ps(1.6e+01f), tmp503);
				__m512 tmp535 = _mm512_fmadd_ps(tmp522, _mm512_set1_ps(1.6e+01f), tmp523);
				__m512 tmp500 = _mm512_add_ps(tmp501, tmp443);
				__m512 tmp520 = _mm512_add_ps(tmp521, tmp451);
				__m512 tmp517 = _mm512_add_ps(tmp518, tmp450);
				__m512 tmp537 = _mm512_add_ps(tmp538, tmp498);
				__m512 tmp499 = _mm512_fmadd_ps(tmp504, _mm512_set1_ps(3.2e+01f), tmp500);
				__m512 tmp519 = _mm512_fmadd_ps(tmp524, _mm512_set1_ps(3.2e+01f), tmp520);
				__m512 tmp510 = _mm512_fmadd_ps(tmp504, _mm512_set1_ps(8e+00f), tmp511);
				__m512 tmp530 = _mm512_fmadd_ps(tmp524, _mm512_set1_ps(8e+00f), tmp531);
				__m512 tmp516 = _mm512_fmadd_ps(tmp508, _mm512_set1_ps(3.2e+01f), tmp517);
				__m512 tmp536 = _mm512_fmadd_ps(tmp528, _mm512_set1_ps(3.2e+01f), tmp537);
				__m512 tmp514 = _mm512_fmadd_ps(tmp504, _mm512_set1_ps(2e+00f), tmp515);
				__m512 tmp534 = _mm512_fmadd_ps(tmp524, _mm512_set1_ps(2e+00f), tmp535);
				__m512 out99 = tmp499;
				__m512 out93 = tmp519;
				__m512 out100 = tmp505;
				__m512 out94 = tmp525;
				__m512 out101 = tmp510;
				__m512 out95 = tmp530;
				__m512 out102 = tmp512;
				__m512 out96 = tmp532;
				__m512 out103 = tmp514;
				__m512 out97 = tmp534;
				__m512 out104 = tmp516;
				__m512 out98 = tmp536;
				out99 = _mm512_max_ps(_mm512_setzero_ps(), out99);
				out93 = _mm512_max_ps(_mm512_setzero_ps(), out93);
				out100 = _mm512_max_ps(_mm512_setzero_ps(), out100);
				out94 = _mm512_max_ps(_mm512_setzero_ps(), out94);
				out101 = _mm512_max_ps(_mm512_setzero_ps(), out101);
				out95 = _mm512_max_ps(_mm512_setzero_ps(), out95);
				out102 = _mm512_max_ps(_mm512_setzero_ps(), out102);
				out96 = _mm512_max_ps(_mm512_setzero_ps(), out96);
				out103 = _mm512_max_ps(_mm512_setzero_ps(), out103);
				out97 = _mm512_max_ps(_mm512_setzero_ps(), out97);
				out104 = _mm512_max_ps(_mm512_setzero_ps(), out104);
				out98 = _mm512_max_ps(_mm512_setzero_ps(), out98);
				out99 = _mm512_add_ps(out99, _mm512_maskz_loadu_ps(4095, datPtr3+1320+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out93 = _mm512_add_ps(out93, _mm512_maskz_loadu_ps(63, datPtr3+96+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out93 = _mm512_add_ps(out93, _mm512_maskz_loadu_ps(4032, datPtr3+696+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out100 = _mm512_add_ps(out100, _mm512_maskz_loadu_ps(4095, datPtr3+1440+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out94 = _mm512_add_ps(out94, _mm512_maskz_loadu_ps(63, datPtr3+216+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out94 = _mm512_add_ps(out94, _mm512_maskz_loadu_ps(4032, datPtr3+816+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out101 = _mm512_add_ps(out101, _mm512_maskz_loadu_ps(4095, datPtr3+1560+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out95 = _mm512_add_ps(out95, _mm512_maskz_loadu_ps(63, datPtr3+336+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out95 = _mm512_add_ps(out95, _mm512_maskz_loadu_ps(4032, datPtr3+936+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out102 = _mm512_add_ps(out102, _mm512_maskz_loadu_ps(4095, datPtr3+1680+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out96 = _mm512_add_ps(out96, _mm512_maskz_loadu_ps(63, datPtr3+456+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out96 = _mm512_add_ps(out96, _mm512_maskz_loadu_ps(4032, datPtr3+1056+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out103 = _mm512_add_ps(out103, _mm512_maskz_loadu_ps(4095, datPtr3+1800+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out97 = _mm512_add_ps(out97, _mm512_maskz_loadu_ps(63, datPtr3+576+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out97 = _mm512_add_ps(out97, _mm512_maskz_loadu_ps(4032, datPtr3+1176+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out104 = _mm512_add_ps(out104, _mm512_maskz_loadu_ps(4095, datPtr3+1920+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out98 = _mm512_add_ps(out98, _mm512_maskz_loadu_ps(63, datPtr3+696+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				__m512 bnMul5 = _mm512_set1_ps(((float*)bnPtr4+(ptrdiff_t)2*(1+68*i8+4*k5+2*l3))[0]);
				__m512 bnAdd5 = _mm512_set1_ps(((float*)bnPtr4+(ptrdiff_t)2*(1+68*i8+4*k5+2*l3))[1]);
				out99 = _mm512_fmadd_ps(out99, bnMul5, bnAdd5);
				out93 = _mm512_fmadd_ps(out93, bnMul4, bnAdd4);
				out100 = _mm512_fmadd_ps(out100, bnMul5, bnAdd5);
				out94 = _mm512_fmadd_ps(out94, bnMul4, bnAdd4);
				out101 = _mm512_fmadd_ps(out101, bnMul5, bnAdd5);
				out95 = _mm512_fmadd_ps(out95, bnMul4, bnAdd4);
				out102 = _mm512_fmadd_ps(out102, bnMul5, bnAdd5);
				out96 = _mm512_fmadd_ps(out96, bnMul4, bnAdd4);
				out103 = _mm512_fmadd_ps(out103, bnMul5, bnAdd5);
				out97 = _mm512_fmadd_ps(out97, bnMul4, bnAdd4);
				out104 = _mm512_fmadd_ps(out104, bnMul5, bnAdd5);
				out98 = _mm512_fmadd_ps(out98, bnMul4, bnAdd4);
				_mm512_mask_storeu_ps(datPtr4+1320+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out99);
				_mm512_mask_storeu_ps(datPtr4+96+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out93);
				_mm512_mask_storeu_ps(datPtr4+696+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4032, out93);
				_mm512_mask_storeu_ps(datPtr4+1440+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out100);
				_mm512_mask_storeu_ps(datPtr4+216+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out94);
				_mm512_mask_storeu_ps(datPtr4+816+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4032, out94);
				_mm512_mask_storeu_ps(datPtr4+1560+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out101);
				_mm512_mask_storeu_ps(datPtr4+336+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out95);
				_mm512_mask_storeu_ps(datPtr4+936+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4032, out95);
				_mm512_mask_storeu_ps(datPtr4+1680+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out102);
				_mm512_mask_storeu_ps(datPtr4+456+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out96);
				_mm512_mask_storeu_ps(datPtr4+1056+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4032, out96);
				_mm512_mask_storeu_ps(datPtr4+1800+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out103);
				_mm512_mask_storeu_ps(datPtr4+576+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out97);
				_mm512_mask_storeu_ps(datPtr4+1176+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4032, out97);
				_mm512_mask_storeu_ps(datPtr4+1920+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out104);
				_mm512_mask_storeu_ps(datPtr4+696+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out98);
				__m512 sf33 = _mm512_loadu_ps(sfPtr2+512+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf34 = _mm512_loadu_ps(sfPtr2+640+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in98 = _mm512_shuffle_f32x4(sf33, sf34, 68);
				__m512 in99 = _mm512_shuffle_f32x4(sf33, sf34, 238);
				__m512 sf35 = _mm512_loadu_ps(sfPtr2+576+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf36 = _mm512_loadu_ps(sfPtr2+704+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in106 = _mm512_shuffle_f32x4(sf35, sf36, 68);
				__m512 in107 = _mm512_shuffle_f32x4(sf35, sf36, 238);
				__m512 sf37 = _mm512_loadu_ps(sfPtr2+44032+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf38 = _mm512_loadu_ps(sfPtr2+44160+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in100 = _mm512_shuffle_f32x4(sf37, sf38, 68);
				__m512 in101 = _mm512_shuffle_f32x4(sf37, sf38, 238);
				__m512 sf39 = _mm512_loadu_ps(sfPtr2+44096+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf40 = _mm512_loadu_ps(sfPtr2+44224+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in108 = _mm512_shuffle_f32x4(sf39, sf40, 68);
				__m512 in109 = _mm512_shuffle_f32x4(sf39, sf40, 238);
				__m512 sf41 = _mm512_loadu_ps(sfPtr2+87552+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf42 = _mm512_loadu_ps(sfPtr2+87680+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in102 = _mm512_shuffle_f32x4(sf41, sf42, 68);
				__m512 in103 = _mm512_shuffle_f32x4(sf41, sf42, 238);
				__m512 sf43 = _mm512_loadu_ps(sfPtr2+87616+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf44 = _mm512_loadu_ps(sfPtr2+87744+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in110 = _mm512_shuffle_f32x4(sf43, sf44, 68);
				__m512 in111 = _mm512_shuffle_f32x4(sf43, sf44, 238);
				__m512 sf45 = _mm512_loadu_ps(sfPtr2+131072+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf46 = _mm512_loadu_ps(sfPtr2+131200+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in104 = _mm512_shuffle_f32x4(sf45, sf46, 68);
				__m512 in105 = _mm512_shuffle_f32x4(sf45, sf46, 238);
				__m512 sf47 = _mm512_loadu_ps(sfPtr2+131136+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 sf48 = _mm512_loadu_ps(sfPtr2+131264+174080*i8+26112*j4+1536*k5+768*l3);
				__m512 in112 = _mm512_shuffle_f32x4(sf47, sf48, 68);
				__m512 in113 = _mm512_shuffle_f32x4(sf47, sf48, 238);
				__m512 tmp595 = _mm512_add_ps(in99, in100);
				__m512 tmp615 = _mm512_add_ps(in107, in108);
				__m512 tmp594 = _mm512_add_ps(in101, in102);
				__m512 tmp614 = _mm512_add_ps(in109, in110);
				__m512 tmp600 = _mm512_sub_ps(in101, in102);
				__m512 tmp620 = _mm512_sub_ps(in109, in110);
				__m512 tmp599 = _mm512_sub_ps(in99, in100);
				__m512 tmp619 = _mm512_sub_ps(in107, in108);
				__m512 tmp596 = _mm512_add_ps(in103, in104);
				__m512 tmp616 = _mm512_add_ps(in111, in112);
				__m512 tmp601 = _mm512_sub_ps(in103, in104);
				__m512 tmp621 = _mm512_sub_ps(in111, in112);
				__m512 tmp598 = _mm512_fmadd_ps(tmp600, _mm512_set1_ps(2e+00f), tmp599);
				__m512 tmp618 = _mm512_fmadd_ps(tmp620, _mm512_set1_ps(2e+00f), tmp619);
				__m512 tmp605 = _mm512_fmadd_ps(tmp600, _mm512_set1_ps(8e+00f), tmp599);
				__m512 tmp625 = _mm512_fmadd_ps(tmp620, _mm512_set1_ps(8e+00f), tmp619);
				__m512 tmp593 = _mm512_add_ps(tmp594, tmp595);
				__m512 tmp613 = _mm512_add_ps(tmp614, tmp615);
				__m512 tmp597 = _mm512_fmadd_ps(tmp601, _mm512_set1_ps(1.6e+01f), tmp598);
				__m512 tmp617 = _mm512_fmadd_ps(tmp621, _mm512_set1_ps(1.6e+01f), tmp618);
				__m512 tmp604 = _mm512_fmadd_ps(tmp601, _mm512_set1_ps(4e+00f), tmp605);
				__m512 tmp624 = _mm512_fmadd_ps(tmp621, _mm512_set1_ps(4e+00f), tmp625);
				__m512 tmp610 = _mm512_add_ps(tmp601, tmp599);
				__m512 tmp630 = _mm512_add_ps(tmp621, tmp619);
				__m512 tmp603 = _mm512_fmadd_ps(tmp594, _mm512_set1_ps(4e+00f), tmp595);
				__m512 tmp623 = _mm512_fmadd_ps(tmp614, _mm512_set1_ps(4e+00f), tmp615);
				__m512 tmp607 = _mm512_fmadd_ps(tmp594, _mm512_set1_ps(1.6e+01f), tmp595);
				__m512 tmp627 = _mm512_fmadd_ps(tmp614, _mm512_set1_ps(1.6e+01f), tmp615);
				__m512 tmp592 = _mm512_add_ps(tmp593, in98);
				__m512 tmp612 = _mm512_add_ps(tmp613, in106);
				__m512 tmp609 = _mm512_add_ps(tmp610, in105);
				__m512 tmp629 = _mm512_add_ps(tmp630, in113);
				__m512 tmp591 = _mm512_fmadd_ps(tmp596, _mm512_set1_ps(3.2e+01f), tmp592);
				__m512 tmp611 = _mm512_fmadd_ps(tmp616, _mm512_set1_ps(3.2e+01f), tmp612);
				__m512 tmp602 = _mm512_fmadd_ps(tmp596, _mm512_set1_ps(8e+00f), tmp603);
				__m512 tmp622 = _mm512_fmadd_ps(tmp616, _mm512_set1_ps(8e+00f), tmp623);
				__m512 tmp608 = _mm512_fmadd_ps(tmp600, _mm512_set1_ps(3.2e+01f), tmp609);
				__m512 tmp628 = _mm512_fmadd_ps(tmp620, _mm512_set1_ps(3.2e+01f), tmp629);
				__m512 tmp606 = _mm512_fmadd_ps(tmp596, _mm512_set1_ps(2e+00f), tmp607);
				__m512 tmp626 = _mm512_fmadd_ps(tmp616, _mm512_set1_ps(2e+00f), tmp627);
				__m512 tmp579 = tmp591;
				__m512 tmp585 = tmp611;
				__m512 tmp580 = tmp597;
				__m512 tmp586 = tmp617;
				__m512 tmp581 = tmp602;
				__m512 tmp587 = tmp622;
				__m512 tmp582 = tmp604;
				__m512 tmp588 = tmp624;
				__m512 tmp583 = tmp606;
				__m512 tmp589 = tmp626;
				__m512 tmp584 = tmp608;
				__m512 tmp590 = tmp628;
				__m512 tmp675 = _mm512_unpacklo_ps(tmp579, tmp580);
				__m512 tmp676 = _mm512_unpackhi_ps(tmp579, tmp580);
				__m512 tmp677 = _mm512_unpacklo_ps(tmp581, tmp582);
				__m512 tmp678 = _mm512_unpackhi_ps(tmp581, tmp582);
				__m512 tmp679 = _mm512_unpacklo_ps(tmp583, tmp584);
				__m512 tmp680 = _mm512_unpackhi_ps(tmp583, tmp584);
				__m512 tmp681 = _mm512_unpacklo_ps(tmp585, tmp586);
				__m512 tmp682 = _mm512_unpackhi_ps(tmp585, tmp586);
				__m512 tmp683 = _mm512_unpacklo_ps(tmp587, tmp588);
				__m512 tmp684 = _mm512_unpackhi_ps(tmp587, tmp588);
				__m512 tmp685 = _mm512_unpacklo_ps(tmp589, tmp590);
				__m512 tmp686 = _mm512_unpackhi_ps(tmp589, tmp590);
				__m512 tmp687 = _mm512_shuffle_ps(tmp675, tmp677, 68);
				__m512 tmp688 = _mm512_shuffle_ps(tmp675, tmp677, 238);
				__m512 tmp689 = _mm512_shuffle_ps(tmp676, tmp678, 68);
				__m512 tmp690 = _mm512_shuffle_ps(tmp676, tmp678, 238);
				__m512 tmp691 = _mm512_shuffle_ps(tmp679, tmp681, 68);
				__m512 tmp692 = _mm512_shuffle_ps(tmp679, tmp681, 238);
				__m512 tmp693 = _mm512_shuffle_ps(tmp680, tmp682, 68);
				__m512 tmp694 = _mm512_shuffle_ps(tmp680, tmp682, 238);
				__m512 tmp695 = _mm512_shuffle_ps(tmp683, tmp685, 68);
				__m512 tmp696 = _mm512_shuffle_ps(tmp683, tmp685, 238);
				__m512 tmp697 = _mm512_shuffle_ps(tmp684, tmp686, 68);
				__m512 tmp698 = _mm512_shuffle_ps(tmp684, tmp686, 238);
				__m512 tmp699 = _mm512_shuffle_f32x4(tmp687, tmp691, 136);
				__m512 tmp700 = _mm512_shuffle_f32x4(tmp687, tmp691, 221);
				__m512 tmp701 = _mm512_shuffle_f32x4(tmp688, tmp692, 136);
				__m512 tmp702 = _mm512_shuffle_f32x4(tmp688, tmp692, 221);
				__m512 tmp703 = _mm512_shuffle_f32x4(tmp689, tmp693, 136);
				__m512 tmp704 = _mm512_shuffle_f32x4(tmp689, tmp693, 221);
				__m512 tmp705 = _mm512_shuffle_f32x4(tmp690, tmp694, 136);
				__m512 tmp706 = _mm512_shuffle_f32x4(tmp690, tmp694, 221);
				__m512 tmp707 = _mm512_shuffle_f32x4(tmp695, tmp695, 136);
				__m512 tmp708 = _mm512_shuffle_f32x4(tmp695, tmp695, 221);
				__m512 tmp709 = _mm512_shuffle_f32x4(tmp696, tmp696, 136);
				__m512 tmp710 = _mm512_shuffle_f32x4(tmp696, tmp696, 221);
				__m512 tmp711 = _mm512_shuffle_f32x4(tmp697, tmp697, 136);
				__m512 tmp712 = _mm512_shuffle_f32x4(tmp697, tmp697, 221);
				__m512 tmp713 = _mm512_shuffle_f32x4(tmp698, tmp698, 136);
				__m512 tmp714 = _mm512_shuffle_f32x4(tmp698, tmp698, 221);
				tmp579 = _mm512_shuffle_f32x4(tmp699, tmp707, 136);
				tmp587 = _mm512_shuffle_f32x4(tmp699, tmp707, 221);
				tmp580 = _mm512_shuffle_f32x4(tmp701, tmp709, 136);
				tmp588 = _mm512_shuffle_f32x4(tmp701, tmp709, 221);
				tmp581 = _mm512_shuffle_f32x4(tmp703, tmp711, 136);
				tmp589 = _mm512_shuffle_f32x4(tmp703, tmp711, 221);
				tmp582 = _mm512_shuffle_f32x4(tmp705, tmp713, 136);
				tmp590 = _mm512_shuffle_f32x4(tmp705, tmp713, 221);
				tmp583 = _mm512_shuffle_f32x4(tmp700, tmp708, 136);
				__m512 tmp631 = _mm512_shuffle_f32x4(tmp700, tmp708, 221);
				tmp584 = _mm512_shuffle_f32x4(tmp702, tmp710, 136);
				__m512 tmp632 = _mm512_shuffle_f32x4(tmp702, tmp710, 221);
				tmp585 = _mm512_shuffle_f32x4(tmp704, tmp712, 136);
				__m512 tmp633 = _mm512_shuffle_f32x4(tmp704, tmp712, 221);
				tmp586 = _mm512_shuffle_f32x4(tmp706, tmp714, 136);
				__m512 tmp634 = _mm512_shuffle_f32x4(tmp706, tmp714, 221);
				__m512 tmp639 = _mm512_add_ps(tmp580, tmp581);
				__m512 tmp659 = _mm512_add_ps(tmp588, tmp589);
				__m512 tmp638 = _mm512_add_ps(tmp582, tmp583);
				__m512 tmp658 = _mm512_add_ps(tmp590, tmp631);
				__m512 tmp644 = _mm512_sub_ps(tmp582, tmp583);
				__m512 tmp664 = _mm512_sub_ps(tmp590, tmp631);
				__m512 tmp643 = _mm512_sub_ps(tmp580, tmp581);
				__m512 tmp663 = _mm512_sub_ps(tmp588, tmp589);
				__m512 tmp640 = _mm512_add_ps(tmp584, tmp585);
				__m512 tmp660 = _mm512_add_ps(tmp632, tmp633);
				__m512 tmp645 = _mm512_sub_ps(tmp584, tmp585);
				__m512 tmp665 = _mm512_sub_ps(tmp632, tmp633);
				__m512 tmp642 = _mm512_fmadd_ps(tmp644, _mm512_set1_ps(2e+00f), tmp643);
				__m512 tmp662 = _mm512_fmadd_ps(tmp664, _mm512_set1_ps(2e+00f), tmp663);
				__m512 tmp649 = _mm512_fmadd_ps(tmp644, _mm512_set1_ps(8e+00f), tmp643);
				__m512 tmp669 = _mm512_fmadd_ps(tmp664, _mm512_set1_ps(8e+00f), tmp663);
				__m512 tmp637 = _mm512_add_ps(tmp638, tmp639);
				__m512 tmp657 = _mm512_add_ps(tmp658, tmp659);
				__m512 tmp641 = _mm512_fmadd_ps(tmp645, _mm512_set1_ps(1.6e+01f), tmp642);
				__m512 tmp661 = _mm512_fmadd_ps(tmp665, _mm512_set1_ps(1.6e+01f), tmp662);
				__m512 tmp648 = _mm512_fmadd_ps(tmp645, _mm512_set1_ps(4e+00f), tmp649);
				__m512 tmp668 = _mm512_fmadd_ps(tmp665, _mm512_set1_ps(4e+00f), tmp669);
				__m512 tmp654 = _mm512_add_ps(tmp645, tmp643);
				__m512 tmp674 = _mm512_add_ps(tmp665, tmp663);
				__m512 tmp647 = _mm512_fmadd_ps(tmp638, _mm512_set1_ps(4e+00f), tmp639);
				__m512 tmp667 = _mm512_fmadd_ps(tmp658, _mm512_set1_ps(4e+00f), tmp659);
				__m512 tmp651 = _mm512_fmadd_ps(tmp638, _mm512_set1_ps(1.6e+01f), tmp639);
				__m512 tmp671 = _mm512_fmadd_ps(tmp658, _mm512_set1_ps(1.6e+01f), tmp659);
				__m512 tmp636 = _mm512_add_ps(tmp637, tmp579);
				__m512 tmp656 = _mm512_add_ps(tmp657, tmp587);
				__m512 tmp653 = _mm512_add_ps(tmp654, tmp586);
				__m512 tmp673 = _mm512_add_ps(tmp674, tmp634);
				__m512 tmp635 = _mm512_fmadd_ps(tmp640, _mm512_set1_ps(3.2e+01f), tmp636);
				__m512 tmp655 = _mm512_fmadd_ps(tmp660, _mm512_set1_ps(3.2e+01f), tmp656);
				__m512 tmp646 = _mm512_fmadd_ps(tmp640, _mm512_set1_ps(8e+00f), tmp647);
				__m512 tmp666 = _mm512_fmadd_ps(tmp660, _mm512_set1_ps(8e+00f), tmp667);
				__m512 tmp652 = _mm512_fmadd_ps(tmp644, _mm512_set1_ps(3.2e+01f), tmp653);
				__m512 tmp672 = _mm512_fmadd_ps(tmp664, _mm512_set1_ps(3.2e+01f), tmp673);
				__m512 tmp650 = _mm512_fmadd_ps(tmp640, _mm512_set1_ps(2e+00f), tmp651);
				__m512 tmp670 = _mm512_fmadd_ps(tmp660, _mm512_set1_ps(2e+00f), tmp671);
				__m512 out105 = tmp635;
				__m512 out111 = tmp655;
				__m512 out106 = tmp641;
				__m512 out112 = tmp661;
				__m512 out107 = tmp646;
				__m512 out113 = tmp666;
				__m512 out108 = tmp648;
				__m512 out114 = tmp668;
				__m512 out109 = tmp650;
				__m512 out115 = tmp670;
				__m512 out110 = tmp652;
				__m512 out116 = tmp672;
				out105 = _mm512_max_ps(_mm512_setzero_ps(), out105);
				out111 = _mm512_max_ps(_mm512_setzero_ps(), out111);
				out106 = _mm512_max_ps(_mm512_setzero_ps(), out106);
				out112 = _mm512_max_ps(_mm512_setzero_ps(), out112);
				out107 = _mm512_max_ps(_mm512_setzero_ps(), out107);
				out113 = _mm512_max_ps(_mm512_setzero_ps(), out113);
				out108 = _mm512_max_ps(_mm512_setzero_ps(), out108);
				out114 = _mm512_max_ps(_mm512_setzero_ps(), out114);
				out109 = _mm512_max_ps(_mm512_setzero_ps(), out109);
				out115 = _mm512_max_ps(_mm512_setzero_ps(), out115);
				out110 = _mm512_max_ps(_mm512_setzero_ps(), out110);
				out116 = _mm512_max_ps(_mm512_setzero_ps(), out116);
				out105 = _mm512_add_ps(out105, _mm512_maskz_loadu_ps(4095, datPtr3+1368+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out111 = _mm512_add_ps(out111, _mm512_maskz_loadu_ps(63, datPtr3+1416+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out111 = _mm512_add_ps(out111, _mm512_maskz_loadu_ps(4032, datPtr3+2016+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out106 = _mm512_add_ps(out106, _mm512_maskz_loadu_ps(4095, datPtr3+1488+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out112 = _mm512_add_ps(out112, _mm512_maskz_loadu_ps(63, datPtr3+1536+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out112 = _mm512_add_ps(out112, _mm512_maskz_loadu_ps(4032, datPtr3+2136+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out107 = _mm512_add_ps(out107, _mm512_maskz_loadu_ps(4095, datPtr3+1608+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out113 = _mm512_add_ps(out113, _mm512_maskz_loadu_ps(63, datPtr3+1656+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out113 = _mm512_add_ps(out113, _mm512_maskz_loadu_ps(4032, datPtr3+2256+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out108 = _mm512_add_ps(out108, _mm512_maskz_loadu_ps(4095, datPtr3+1728+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out114 = _mm512_add_ps(out114, _mm512_maskz_loadu_ps(63, datPtr3+1776+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out114 = _mm512_add_ps(out114, _mm512_maskz_loadu_ps(4032, datPtr3+2376+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out109 = _mm512_add_ps(out109, _mm512_maskz_loadu_ps(4095, datPtr3+1848+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out115 = _mm512_add_ps(out115, _mm512_maskz_loadu_ps(63, datPtr3+1896+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out115 = _mm512_add_ps(out115, _mm512_maskz_loadu_ps(4032, datPtr3+2496+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out110 = _mm512_add_ps(out110, _mm512_maskz_loadu_ps(4095, datPtr3+1968+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out116 = _mm512_add_ps(out116, _mm512_maskz_loadu_ps(63, datPtr3+2016+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3));
				out105 = _mm512_fmadd_ps(out105, bnMul5, bnAdd5);
				out111 = _mm512_fmadd_ps(out111, bnMul5, bnAdd5);
				out106 = _mm512_fmadd_ps(out106, bnMul5, bnAdd5);
				out112 = _mm512_fmadd_ps(out112, bnMul5, bnAdd5);
				out107 = _mm512_fmadd_ps(out107, bnMul5, bnAdd5);
				out113 = _mm512_fmadd_ps(out113, bnMul5, bnAdd5);
				out108 = _mm512_fmadd_ps(out108, bnMul5, bnAdd5);
				out114 = _mm512_fmadd_ps(out114, bnMul5, bnAdd5);
				out109 = _mm512_fmadd_ps(out109, bnMul5, bnAdd5);
				out115 = _mm512_fmadd_ps(out115, bnMul5, bnAdd5);
				out110 = _mm512_fmadd_ps(out110, bnMul5, bnAdd5);
				out116 = _mm512_fmadd_ps(out116, bnMul5, bnAdd5);
				_mm512_mask_storeu_ps(datPtr4+1368+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out105);
				_mm512_mask_storeu_ps(datPtr4+1416+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out111);
				_mm512_mask_storeu_ps(datPtr4+2016+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4032, out111);
				_mm512_mask_storeu_ps(datPtr4+1488+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out106);
				_mm512_mask_storeu_ps(datPtr4+1536+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out112);
				_mm512_mask_storeu_ps(datPtr4+2136+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4032, out112);
				_mm512_mask_storeu_ps(datPtr4+1608+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out107);
				_mm512_mask_storeu_ps(datPtr4+1656+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out113);
				_mm512_mask_storeu_ps(datPtr4+2256+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4032, out113);
				_mm512_mask_storeu_ps(datPtr4+1728+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out108);
				_mm512_mask_storeu_ps(datPtr4+1776+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out114);
				_mm512_mask_storeu_ps(datPtr4+2376+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4032, out114);
				_mm512_mask_storeu_ps(datPtr4+1848+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out109);
				_mm512_mask_storeu_ps(datPtr4+1896+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out115);
				_mm512_mask_storeu_ps(datPtr4+2496+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4032, out115);
				_mm512_mask_storeu_ps(datPtr4+1968+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 4095, out110);
				_mm512_mask_storeu_ps(datPtr4+2016+89760*i8+120*toH1+4*toW1+5280*k5+2640*l3, 63, out116);
			}
		}
		++j4;
		rel2 = 1;
	}
	ptrdiff_t toH2 = base2+6;
	ptrdiff_t toW2 = 6;
	ptrdiff_t k6 = 17*w4;
	for (; k6 != 17; ++k6) {
		ptrdiff_t l4 = 0;
		for (; l4 != 4; ++l4) {
			__m512 sf49 = _mm512_loadu_ps(sfPtr2+0+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 sf50 = _mm512_loadu_ps(sfPtr2+128+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 in114 = _mm512_shuffle_f32x4(sf49, sf50, 68);
			__m512 in115 = _mm512_shuffle_f32x4(sf49, sf50, 238);
			__m512 sf51 = _mm512_loadu_ps(sfPtr2+64+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 sf52 = _mm512_loadu_ps(sfPtr2+192+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 in122 = _mm512_shuffle_f32x4(sf51, sf52, 68);
			__m512 in123 = _mm512_shuffle_f32x4(sf51, sf52, 238);
			__m512 sf53 = _mm512_loadu_ps(sfPtr2+43520+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 sf54 = _mm512_loadu_ps(sfPtr2+43648+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 in116 = _mm512_shuffle_f32x4(sf53, sf54, 68);
			__m512 in117 = _mm512_shuffle_f32x4(sf53, sf54, 238);
			__m512 sf55 = _mm512_loadu_ps(sfPtr2+43584+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 sf56 = _mm512_loadu_ps(sfPtr2+43712+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 in124 = _mm512_shuffle_f32x4(sf55, sf56, 68);
			__m512 in125 = _mm512_shuffle_f32x4(sf55, sf56, 238);
			__m512 sf57 = _mm512_loadu_ps(sfPtr2+87040+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 sf58 = _mm512_loadu_ps(sfPtr2+87168+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 in118 = _mm512_shuffle_f32x4(sf57, sf58, 68);
			__m512 in119 = _mm512_shuffle_f32x4(sf57, sf58, 238);
			__m512 sf59 = _mm512_loadu_ps(sfPtr2+87104+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 sf60 = _mm512_loadu_ps(sfPtr2+87232+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 in126 = _mm512_shuffle_f32x4(sf59, sf60, 68);
			__m512 in127 = _mm512_shuffle_f32x4(sf59, sf60, 238);
			__m512 sf61 = _mm512_loadu_ps(sfPtr2+130560+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 sf62 = _mm512_loadu_ps(sfPtr2+130688+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 in120 = _mm512_shuffle_f32x4(sf61, sf62, 68);
			__m512 in121 = _mm512_shuffle_f32x4(sf61, sf62, 238);
			__m512 sf63 = _mm512_loadu_ps(sfPtr2+130624+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 sf64 = _mm512_loadu_ps(sfPtr2+130752+174080*i8+26112*j4+1024*k6+256*l4);
			__m512 in128 = _mm512_shuffle_f32x4(sf63, sf64, 68);
			__m512 in129 = _mm512_shuffle_f32x4(sf63, sf64, 238);
			__m512 tmp731 = _mm512_add_ps(in115, in116);
			__m512 tmp751 = _mm512_add_ps(in123, in124);
			__m512 tmp730 = _mm512_add_ps(in117, in118);
			__m512 tmp750 = _mm512_add_ps(in125, in126);
			__m512 tmp736 = _mm512_sub_ps(in117, in118);
			__m512 tmp756 = _mm512_sub_ps(in125, in126);
			__m512 tmp735 = _mm512_sub_ps(in115, in116);
			__m512 tmp755 = _mm512_sub_ps(in123, in124);
			__m512 tmp732 = _mm512_add_ps(in119, in120);
			__m512 tmp752 = _mm512_add_ps(in127, in128);
			__m512 tmp737 = _mm512_sub_ps(in119, in120);
			__m512 tmp757 = _mm512_sub_ps(in127, in128);
			__m512 tmp734 = _mm512_fmadd_ps(tmp736, _mm512_set1_ps(2e+00f), tmp735);
			__m512 tmp754 = _mm512_fmadd_ps(tmp756, _mm512_set1_ps(2e+00f), tmp755);
			__m512 tmp741 = _mm512_fmadd_ps(tmp736, _mm512_set1_ps(8e+00f), tmp735);
			__m512 tmp761 = _mm512_fmadd_ps(tmp756, _mm512_set1_ps(8e+00f), tmp755);
			__m512 tmp729 = _mm512_add_ps(tmp730, tmp731);
			__m512 tmp749 = _mm512_add_ps(tmp750, tmp751);
			__m512 tmp733 = _mm512_fmadd_ps(tmp737, _mm512_set1_ps(1.6e+01f), tmp734);
			__m512 tmp753 = _mm512_fmadd_ps(tmp757, _mm512_set1_ps(1.6e+01f), tmp754);
			__m512 tmp740 = _mm512_fmadd_ps(tmp737, _mm512_set1_ps(4e+00f), tmp741);
			__m512 tmp760 = _mm512_fmadd_ps(tmp757, _mm512_set1_ps(4e+00f), tmp761);
			__m512 tmp746 = _mm512_add_ps(tmp737, tmp735);
			__m512 tmp766 = _mm512_add_ps(tmp757, tmp755);
			__m512 tmp739 = _mm512_fmadd_ps(tmp730, _mm512_set1_ps(4e+00f), tmp731);
			__m512 tmp759 = _mm512_fmadd_ps(tmp750, _mm512_set1_ps(4e+00f), tmp751);
			__m512 tmp743 = _mm512_fmadd_ps(tmp730, _mm512_set1_ps(1.6e+01f), tmp731);
			__m512 tmp763 = _mm512_fmadd_ps(tmp750, _mm512_set1_ps(1.6e+01f), tmp751);
			__m512 tmp728 = _mm512_add_ps(tmp729, in114);
			__m512 tmp748 = _mm512_add_ps(tmp749, in122);
			__m512 tmp745 = _mm512_add_ps(tmp746, in121);
			__m512 tmp765 = _mm512_add_ps(tmp766, in129);
			__m512 tmp727 = _mm512_fmadd_ps(tmp732, _mm512_set1_ps(3.2e+01f), tmp728);
			__m512 tmp747 = _mm512_fmadd_ps(tmp752, _mm512_set1_ps(3.2e+01f), tmp748);
			__m512 tmp738 = _mm512_fmadd_ps(tmp732, _mm512_set1_ps(8e+00f), tmp739);
			__m512 tmp758 = _mm512_fmadd_ps(tmp752, _mm512_set1_ps(8e+00f), tmp759);
			__m512 tmp744 = _mm512_fmadd_ps(tmp736, _mm512_set1_ps(3.2e+01f), tmp745);
			__m512 tmp764 = _mm512_fmadd_ps(tmp756, _mm512_set1_ps(3.2e+01f), tmp765);
			__m512 tmp742 = _mm512_fmadd_ps(tmp732, _mm512_set1_ps(2e+00f), tmp743);
			__m512 tmp762 = _mm512_fmadd_ps(tmp752, _mm512_set1_ps(2e+00f), tmp763);
			__m512 tmp715 = tmp727;
			__m512 tmp721 = tmp747;
			__m512 tmp716 = tmp733;
			__m512 tmp722 = tmp753;
			__m512 tmp717 = tmp738;
			__m512 tmp723 = tmp758;
			__m512 tmp718 = tmp740;
			__m512 tmp724 = tmp760;
			__m512 tmp719 = tmp742;
			__m512 tmp725 = tmp762;
			__m512 tmp720 = tmp744;
			__m512 tmp726 = tmp764;
			__m512 tmp805 = _mm512_unpacklo_ps(tmp715, tmp716);
			__m512 tmp806 = _mm512_unpackhi_ps(tmp715, tmp716);
			__m512 tmp807 = _mm512_unpacklo_ps(tmp717, tmp718);
			__m512 tmp808 = _mm512_unpackhi_ps(tmp717, tmp718);
			__m512 tmp809 = _mm512_unpacklo_ps(tmp719, tmp720);
			__m512 tmp810 = _mm512_unpackhi_ps(tmp719, tmp720);
			__m512 tmp811 = _mm512_unpacklo_ps(tmp721, tmp722);
			__m512 tmp812 = _mm512_unpackhi_ps(tmp721, tmp722);
			__m512 tmp813 = _mm512_unpacklo_ps(tmp723, tmp724);
			__m512 tmp814 = _mm512_unpackhi_ps(tmp723, tmp724);
			__m512 tmp815 = _mm512_unpacklo_ps(tmp725, tmp726);
			__m512 tmp816 = _mm512_unpackhi_ps(tmp725, tmp726);
			__m512 tmp817 = _mm512_shuffle_ps(tmp805, tmp807, 68);
			__m512 tmp818 = _mm512_shuffle_ps(tmp805, tmp807, 238);
			__m512 tmp819 = _mm512_shuffle_ps(tmp806, tmp808, 68);
			__m512 tmp820 = _mm512_shuffle_ps(tmp806, tmp808, 238);
			__m512 tmp821 = _mm512_shuffle_ps(tmp809, tmp811, 68);
			__m512 tmp822 = _mm512_shuffle_ps(tmp809, tmp811, 238);
			__m512 tmp823 = _mm512_shuffle_ps(tmp810, tmp812, 68);
			__m512 tmp824 = _mm512_shuffle_ps(tmp810, tmp812, 238);
			__m512 tmp825 = _mm512_shuffle_ps(tmp813, tmp815, 68);
			__m512 tmp826 = _mm512_shuffle_ps(tmp813, tmp815, 238);
			__m512 tmp827 = _mm512_shuffle_ps(tmp814, tmp816, 68);
			__m512 tmp828 = _mm512_shuffle_ps(tmp814, tmp816, 238);
			__m512 tmp829 = _mm512_shuffle_f32x4(tmp817, tmp821, 136);
			__m512 tmp830 = _mm512_shuffle_f32x4(tmp817, tmp821, 221);
			__m512 tmp831 = _mm512_shuffle_f32x4(tmp818, tmp822, 136);
			__m512 tmp832 = _mm512_shuffle_f32x4(tmp818, tmp822, 221);
			__m512 tmp833 = _mm512_shuffle_f32x4(tmp819, tmp823, 136);
			__m512 tmp834 = _mm512_shuffle_f32x4(tmp819, tmp823, 221);
			__m512 tmp835 = _mm512_shuffle_f32x4(tmp820, tmp824, 136);
			__m512 tmp836 = _mm512_shuffle_f32x4(tmp820, tmp824, 221);
			__m512 tmp837 = _mm512_shuffle_f32x4(tmp825, tmp825, 136);
			__m512 tmp838 = _mm512_shuffle_f32x4(tmp825, tmp825, 221);
			__m512 tmp839 = _mm512_shuffle_f32x4(tmp826, tmp826, 136);
			__m512 tmp840 = _mm512_shuffle_f32x4(tmp826, tmp826, 221);
			__m512 tmp841 = _mm512_shuffle_f32x4(tmp827, tmp827, 136);
			__m512 tmp842 = _mm512_shuffle_f32x4(tmp827, tmp827, 221);
			__m512 tmp843 = _mm512_shuffle_f32x4(tmp828, tmp828, 136);
			__m512 tmp844 = _mm512_shuffle_f32x4(tmp828, tmp828, 221);
			tmp715 = _mm512_shuffle_f32x4(tmp829, tmp837, 136);
			tmp723 = _mm512_shuffle_f32x4(tmp829, tmp837, 221);
			tmp716 = _mm512_shuffle_f32x4(tmp831, tmp839, 136);
			tmp724 = _mm512_shuffle_f32x4(tmp831, tmp839, 221);
			tmp717 = _mm512_shuffle_f32x4(tmp833, tmp841, 136);
			tmp725 = _mm512_shuffle_f32x4(tmp833, tmp841, 221);
			tmp718 = _mm512_shuffle_f32x4(tmp835, tmp843, 136);
			tmp726 = _mm512_shuffle_f32x4(tmp835, tmp843, 221);
			tmp719 = _mm512_shuffle_f32x4(tmp830, tmp838, 136);
			__m512 tmp767 = _mm512_shuffle_f32x4(tmp830, tmp838, 221);
			tmp720 = _mm512_shuffle_f32x4(tmp832, tmp840, 136);
			__m512 tmp768 = _mm512_shuffle_f32x4(tmp832, tmp840, 221);
			tmp721 = _mm512_shuffle_f32x4(tmp834, tmp842, 136);
			__m512 tmp769 = _mm512_shuffle_f32x4(tmp834, tmp842, 221);
			tmp722 = _mm512_shuffle_f32x4(tmp836, tmp844, 136);
			__m512 tmp770 = _mm512_shuffle_f32x4(tmp836, tmp844, 221);
			(void)tmp722;
			(void)tmp770;
			__m512 tmp775 = _mm512_add_ps(tmp716, tmp717);
			__m512 tmp792 = _mm512_add_ps(tmp724, tmp725);
			__m512 tmp774 = _mm512_add_ps(tmp718, tmp719);
			__m512 tmp791 = _mm512_add_ps(tmp726, tmp767);
			__m512 tmp780 = _mm512_sub_ps(tmp718, tmp719);
			__m512 tmp797 = _mm512_sub_ps(tmp726, tmp767);
			__m512 tmp779 = _mm512_sub_ps(tmp716, tmp717);
			__m512 tmp796 = _mm512_sub_ps(tmp724, tmp725);
			__m512 tmp776 = _mm512_add_ps(tmp720, tmp721);
			__m512 tmp793 = _mm512_add_ps(tmp768, tmp769);
			__m512 tmp781 = _mm512_sub_ps(tmp720, tmp721);
			__m512 tmp798 = _mm512_sub_ps(tmp768, tmp769);
			__m512 tmp778 = _mm512_fmadd_ps(tmp780, _mm512_set1_ps(2e+00f), tmp779);
			__m512 tmp795 = _mm512_fmadd_ps(tmp797, _mm512_set1_ps(2e+00f), tmp796);
			__m512 tmp785 = _mm512_fmadd_ps(tmp780, _mm512_set1_ps(8e+00f), tmp779);
			__m512 tmp802 = _mm512_fmadd_ps(tmp797, _mm512_set1_ps(8e+00f), tmp796);
			__m512 tmp773 = _mm512_add_ps(tmp774, tmp775);
			__m512 tmp790 = _mm512_add_ps(tmp791, tmp792);
			__m512 tmp777 = _mm512_fmadd_ps(tmp781, _mm512_set1_ps(1.6e+01f), tmp778);
			__m512 tmp794 = _mm512_fmadd_ps(tmp798, _mm512_set1_ps(1.6e+01f), tmp795);
			__m512 tmp784 = _mm512_fmadd_ps(tmp781, _mm512_set1_ps(4e+00f), tmp785);
			__m512 tmp801 = _mm512_fmadd_ps(tmp798, _mm512_set1_ps(4e+00f), tmp802);
			__m512 tmp783 = _mm512_fmadd_ps(tmp774, _mm512_set1_ps(4e+00f), tmp775);
			__m512 tmp800 = _mm512_fmadd_ps(tmp791, _mm512_set1_ps(4e+00f), tmp792);
			__m512 tmp787 = _mm512_fmadd_ps(tmp774, _mm512_set1_ps(1.6e+01f), tmp775);
			__m512 tmp804 = _mm512_fmadd_ps(tmp791, _mm512_set1_ps(1.6e+01f), tmp792);
			__m512 tmp772 = _mm512_add_ps(tmp773, tmp715);
			__m512 tmp789 = _mm512_add_ps(tmp790, tmp723);
			__m512 tmp771 = _mm512_fmadd_ps(tmp776, _mm512_set1_ps(3.2e+01f), tmp772);
			__m512 tmp788 = _mm512_fmadd_ps(tmp793, _mm512_set1_ps(3.2e+01f), tmp789);
			__m512 tmp782 = _mm512_fmadd_ps(tmp776, _mm512_set1_ps(8e+00f), tmp783);
			__m512 tmp799 = _mm512_fmadd_ps(tmp793, _mm512_set1_ps(8e+00f), tmp800);
			__m512 tmp786 = _mm512_fmadd_ps(tmp776, _mm512_set1_ps(2e+00f), tmp787);
			__m512 tmp803 = _mm512_fmadd_ps(tmp793, _mm512_set1_ps(2e+00f), tmp804);
			__m512 out117 = tmp771;
			__m512 out122 = tmp788;
			__m512 out118 = tmp777;
			__m512 out123 = tmp794;
			__m512 out119 = tmp782;
			__m512 out124 = tmp799;
			__m512 out120 = tmp784;
			__m512 out125 = tmp801;
			__m512 out121 = tmp786;
			__m512 out126 = tmp803;
			out117 = _mm512_max_ps(_mm512_setzero_ps(), out117);
			out122 = _mm512_max_ps(_mm512_setzero_ps(), out122);
			out118 = _mm512_max_ps(_mm512_setzero_ps(), out118);
			out123 = _mm512_max_ps(_mm512_setzero_ps(), out123);
			out119 = _mm512_max_ps(_mm512_setzero_ps(), out119);
			out124 = _mm512_max_ps(_mm512_setzero_ps(), out124);
			out120 = _mm512_max_ps(_mm512_setzero_ps(), out120);
			out125 = _mm512_max_ps(_mm512_setzero_ps(), out125);
			out121 = _mm512_max_ps(_mm512_setzero_ps(), out121);
			out126 = _mm512_max_ps(_mm512_setzero_ps(), out126);
			out117 = _mm512_add_ps(out117, _mm512_maskz_loadu_ps(4095, datPtr3+0+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4));
			out122 = _mm512_add_ps(out122, _mm512_maskz_loadu_ps(4095, datPtr3+48+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4));
			out118 = _mm512_add_ps(out118, _mm512_maskz_loadu_ps(4095, datPtr3+120+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4));
			out123 = _mm512_add_ps(out123, _mm512_maskz_loadu_ps(4095, datPtr3+168+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4));
			out119 = _mm512_add_ps(out119, _mm512_maskz_loadu_ps(4095, datPtr3+240+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4));
			out124 = _mm512_add_ps(out124, _mm512_maskz_loadu_ps(4095, datPtr3+288+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4));
			out120 = _mm512_add_ps(out120, _mm512_maskz_loadu_ps(4095, datPtr3+360+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4));
			out125 = _mm512_add_ps(out125, _mm512_maskz_loadu_ps(4095, datPtr3+408+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4));
			out121 = _mm512_add_ps(out121, _mm512_maskz_loadu_ps(4095, datPtr3+480+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4));
			out126 = _mm512_add_ps(out126, _mm512_maskz_loadu_ps(4095, datPtr3+528+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4));
			__m512 bnMul6 = _mm512_set1_ps(((float*)bnPtr4+(ptrdiff_t)2*(0+68*i8+4*k6+1*l4))[0]);
			__m512 bnAdd6 = _mm512_set1_ps(((float*)bnPtr4+(ptrdiff_t)2*(0+68*i8+4*k6+1*l4))[1]);
			out117 = _mm512_fmadd_ps(out117, bnMul6, bnAdd6);
			out122 = _mm512_fmadd_ps(out122, bnMul6, bnAdd6);
			out118 = _mm512_fmadd_ps(out118, bnMul6, bnAdd6);
			out123 = _mm512_fmadd_ps(out123, bnMul6, bnAdd6);
			out119 = _mm512_fmadd_ps(out119, bnMul6, bnAdd6);
			out124 = _mm512_fmadd_ps(out124, bnMul6, bnAdd6);
			out120 = _mm512_fmadd_ps(out120, bnMul6, bnAdd6);
			out125 = _mm512_fmadd_ps(out125, bnMul6, bnAdd6);
			out121 = _mm512_fmadd_ps(out121, bnMul6, bnAdd6);
			out126 = _mm512_fmadd_ps(out126, bnMul6, bnAdd6);
			_mm512_mask_storeu_ps(datPtr4+0+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4, 4095, out117);
			_mm512_mask_storeu_ps(datPtr4+48+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4, 4095, out122);
			_mm512_mask_storeu_ps(datPtr4+120+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4, 4095, out118);
			_mm512_mask_storeu_ps(datPtr4+168+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4, 4095, out123);
			_mm512_mask_storeu_ps(datPtr4+240+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4, 4095, out119);
			_mm512_mask_storeu_ps(datPtr4+288+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4, 4095, out124);
			_mm512_mask_storeu_ps(datPtr4+360+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4, 4095, out120);
			_mm512_mask_storeu_ps(datPtr4+408+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4, 4095, out125);
			_mm512_mask_storeu_ps(datPtr4+480+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4, 4095, out121);
			_mm512_mask_storeu_ps(datPtr4+528+89760*i8+120*toH2+4*toW2+5280*k6+1320*l4, 4095, out126);
		}
	}
	++j4;
}

static void Example28ThreeConsumeSums1(Example28ThreaderTeam1* team17, char** tensors7) {
	Example28ThreaderTask1 task11;
	task11.callee1 = Example28ThreeConsumeSums1Callee1;
	task11.any1 = tensors7;
	task11.nd1 = 3;
	task11.hull1[0] = 1;
	task11.hull1[1] = 1;
	task11.hull1[2] = 1;
	Example28ThreaderDo1(team17, &task11);
}

struct Example28Net {
	char* alloc1;
	char* align1;
};

void Example28NetDestroy(Example28Net* net2) {
	free(net2->alloc1);
	free(net2);
}

char* Example28NetCreate(
	Example28Net** net1,
	Example28Params* params1,
	ptrdiff_t threads1
) {
	if (__builtin_expect(!__builtin_cpu_supports("avx512f"), 0)) {
		return Example28Errmsg1(__LINE__, "CPU does not support AVX512F");
	}
	char* alloc3 = malloc(366911);
	if (__builtin_expect(!alloc3, 0)) {
		return Example28Errmsg1(__LINE__, "errno %d", errno);
	}
	char* align3 = (void*)(((size_t)alloc3+63)&-64);
	char* tmpAlloc1 = malloc(991);
	if (__builtin_expect(!tmpAlloc1, 0)) {
		char* msg6 = Example28Errmsg1(__LINE__, "errno %d", errno);
		free(alloc3);
		return msg6;
	}
	char* tmpAlign1 = (void*)(((size_t)tmpAlloc1+63)&-64);
	Example28ThreaderTeam1* team12 = 0;
	char* err8 = Example28ThreaderCreate1(&team12, threads1);
	if (__builtin_expect(!!err8, 0)) {
		free(tmpAlloc1);
		free(alloc3);
		return err8;
	}
	{
		Example28BnSimplify1(
			params1->bn1Means,
			params1->bn1Variances,
			params1->bn1Scales,
			params1->bn1Shifts,
			align3+0
		);
		Example28BnSimplify2(
			params1->bn4Means,
			params1->bn4Variances,
			params1->bn4Scales,
			params1->bn4Shifts,
			align3+384
		);
		Example28BnSimplify1(
			params1->bn2Means,
			params1->bn2Variances,
			params1->bn2Scales,
			params1->bn2Shifts,
			tmpAlign1+0
		);
		Example28BnSimplify2(
			params1->bn3Means,
			params1->bn3Variances,
			params1->bn3Scales,
			params1->bn3Shifts,
			tmpAlign1+384
		);
		char* tensors12[] = {
			(char*)params1->convWeights,
			(char*)params1->convBiases,
			tmpAlign1+0,
			tmpAlign1+384,
			align3+960
		};
		Example28ThreeArrangeFilts1(team12, tensors12);
	}
	Example28ThreaderDestroy1(team12);
	free(tmpAlloc1);
	Example28Net* net5 = malloc(sizeof(Example28Net));
	if (__builtin_expect(!net5, 0)) {
		char* msg7 = Example28Errmsg1(__LINE__, "errno %d", errno);
		free(alloc3);
		return msg7;
	}
	net5->alloc1 = alloc3;
	net5->align1 = align3;
	*net1 = net5;
	return 0;
}

struct Example28Engine {
	Example28Net* net3;
	Example28ThreaderTeam1* team11;
	char* alloc2;
	char* align2;
};

char* Example28EnginePthreadT(
	Example28Engine* eng2,
	ptrdiff_t idx2,
	pthread_t* to1
) {
	return Example28ThreaderPthreadT1(to1, eng2->team11, idx2);
}

void Example28EngineDestroy(Example28Engine* eng3) {
	Example28ThreaderDestroy1(eng3->team11);
	free(eng3->alloc2);
	free(eng3);
}

char* Example28EngineCreate(
	Example28Engine** eng4,
	Example28Net* net4,
	ptrdiff_t threads2
) {
	Example28Engine* eng5 = malloc(sizeof(Example28Engine));
	if (__builtin_expect(!eng5, 0)) {
		return Example28Errmsg1(__LINE__, "errno %d", errno);
	}
	char* alloc4 = malloc(281663);
	if (__builtin_expect(!alloc4, 0)) {
		char* msg5 = Example28Errmsg1(__LINE__, "errno %d", errno);
		free(eng5);
		return msg5;
	}
	eng5->alloc2 = alloc4;
	eng5->align2 = (void*)(((size_t)alloc4+63)&-64);
	char* err7 = Example28ThreaderCreate1(&eng5->team11, threads2);
	if (__builtin_expect(!!err7, 0)) {
		free(eng5);
		free(alloc4);
		return err7;
	}
	eng5->net3 = net4;
	*eng4 = eng5;
	return 0;
}

void Example28EngineInference(
	Example28Engine* eng1,
	float* bn4Data,
	float* in1Data,
	float* in2Data,
	float* in3Data
) {
	char* netAlign1 = eng1->net3->align1;
	Example28ThreaderTeam1* team14 = eng1->team11;
	char* align4 = eng1->align2;
	{
		char* tensors9[] = {
			(char*)in1Data,
			netAlign1+0,
			(char*)in2Data,
			align4+0
		};
		Example28ThreeArrangeDats1(team14, tensors9);
		char* tensors10[] = {
			netAlign1+960,
			align4+0,
			align4+107520
		};
		Example28ThreeProduceSums1(team14, tensors10);
		char* tensors11[] = {
			align4+107520,
			(char*)in3Data,
			netAlign1+384,
			(char*)bn4Data
		};
		Example28ThreeConsumeSums1(team14, tensors11);
	}
}

// End of file.
