/*----------------------------------------------------------------------------
 
  +----------------------------------------------+
  | Written by Adrian Wills,                     |
  |            School of Elec. Eng. & Comp. Sci. |
  |            University of Newcastle,          |
  |            Callaghan, NSW, 2308, AUSTRALIA   |
  |                                              |
  | Last Revised  25 May 2007.                   |
  |                                              |
  | Copyright (C) Adrian Wills.                  |
  +----------------------------------------------+
 
The current version of this software is free of charge and 
openly distributed, BUT PLEASE NOTE:

This software must be referenced when used in a published work.

This software may not be re-distributed as a part of a commercial product. 
If you distribute it in a non-commercial products, please contact me first, 
to make sure you ship the most recent version.

This software is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

IF IT FAILS TO WORK, IT'S YOUR LOSS AND YOUR PROBLEM.

--------------------------------------------------------------------------------*/
#ifndef QPAS_SUB_H_
#define QPAS_SUB_H_

#define INT_INT

#ifdef INT_INT
#define varint int
#endif

#ifdef INT_LONG_INT
#define varint long int
#endif

#ifdef INT_LONG_LONG_INT
#define varint long long int
#endif

void printvector(const int size, const float * vec, const char * name);

void printmatrix(const int m, const int n, const float * A, const int lda, const char * name);


/* The only ruotine in qp_sub.c */
varint qpas_sub_noblas( varint n,
varint me,
varint mc,
varint nl,
varint nu,
float * H,
float * f,
float * A,
float * bin,
float * lin,
float * uin,
float * x,
float * lm,
varint display,
varint * numits,
varint * numadd,
varint * numdrop
);

#endif
