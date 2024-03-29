/* Include files */

#include "flying_ip_1_sfun.h"
#include "flying_ip_1_sfun_debug_macros.h"
#include "c1_flying_ip_1.h"
#include "c2_flying_ip_1.h"
#include "c3_flying_ip_1.h"
#include "c4_flying_ip_1.h"
#include "c5_flying_ip_1.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _flying_ip_1MachineNumber_;

/* Function Declarations */

/* Function Definitions */
void flying_ip_1_initializer(void)
{
}

void flying_ip_1_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_flying_ip_1_method_dispatcher(SimStruct *simstructPtr, unsigned
  int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_flying_ip_1_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_flying_ip_1_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_flying_ip_1_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_flying_ip_1_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_flying_ip_1_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_flying_ip_1_process_check_sum_call( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1677043742U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(161660708U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1163695572U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1028426852U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2575812752U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(657605657U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2403933148U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2145019398U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_flying_ip_1_get_check_sum(mxArray *plhs[]);
          sf_c1_flying_ip_1_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_flying_ip_1_get_check_sum(mxArray *plhs[]);
          sf_c2_flying_ip_1_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_flying_ip_1_get_check_sum(mxArray *plhs[]);
          sf_c3_flying_ip_1_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_flying_ip_1_get_check_sum(mxArray *plhs[]);
          sf_c4_flying_ip_1_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_flying_ip_1_get_check_sum(mxArray *plhs[]);
          sf_c5_flying_ip_1_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(814460797U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(400623215U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1072597456U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1176453921U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2916171531U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1524614978U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1132629542U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3269819244U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_flying_ip_1_autoinheritance_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(aiChksum, "NGx19SVAMLtfrCLL1MCpsE") == 0) {
          extern mxArray *sf_c1_flying_ip_1_get_autoinheritance_info(void);
          plhs[0] = sf_c1_flying_ip_1_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "iO7Pdvp7FkuyCXerrr8VFE") == 0) {
          extern mxArray *sf_c2_flying_ip_1_get_autoinheritance_info(void);
          plhs[0] = sf_c2_flying_ip_1_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "TP3RSg5ilzLO7mEDp9vxGC") == 0) {
          extern mxArray *sf_c3_flying_ip_1_get_autoinheritance_info(void);
          plhs[0] = sf_c3_flying_ip_1_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "PptNhv7tGF5mVazQ29mF0C") == 0) {
          extern mxArray *sf_c4_flying_ip_1_get_autoinheritance_info(void);
          plhs[0] = sf_c4_flying_ip_1_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "D9EXla7lqPFF7LXObRXhWE") == 0) {
          extern mxArray *sf_c5_flying_ip_1_get_autoinheritance_info(void);
          plhs[0] = sf_c5_flying_ip_1_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_flying_ip_1_get_eml_resolved_functions_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray *sf_c1_flying_ip_1_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_flying_ip_1_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray *sf_c2_flying_ip_1_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_flying_ip_1_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray *sf_c3_flying_ip_1_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_flying_ip_1_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray *sf_c4_flying_ip_1_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_flying_ip_1_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray *sf_c5_flying_ip_1_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_flying_ip_1_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_flying_ip_1_third_party_uses_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "WrmUVbNke2nSoWRnznfmtE") == 0) {
          extern mxArray *sf_c1_flying_ip_1_third_party_uses_info(void);
          plhs[0] = sf_c1_flying_ip_1_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "2wDkw3AjsRIiXazztPmY9C") == 0) {
          extern mxArray *sf_c2_flying_ip_1_third_party_uses_info(void);
          plhs[0] = sf_c2_flying_ip_1_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "3rwZHnUzbQAN5nKWURoC5E") == 0) {
          extern mxArray *sf_c3_flying_ip_1_third_party_uses_info(void);
          plhs[0] = sf_c3_flying_ip_1_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "ufK10B2QEcpQZqIQ1RwsF") == 0) {
          extern mxArray *sf_c4_flying_ip_1_third_party_uses_info(void);
          plhs[0] = sf_c4_flying_ip_1_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "xsI6r66MQduLdhBqKZoRaB") == 0) {
          extern mxArray *sf_c5_flying_ip_1_third_party_uses_info(void);
          plhs[0] = sf_c5_flying_ip_1_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_flying_ip_1_updateBuildInfo_args_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "WrmUVbNke2nSoWRnznfmtE") == 0) {
          extern mxArray *sf_c1_flying_ip_1_updateBuildInfo_args_info(void);
          plhs[0] = sf_c1_flying_ip_1_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "2wDkw3AjsRIiXazztPmY9C") == 0) {
          extern mxArray *sf_c2_flying_ip_1_updateBuildInfo_args_info(void);
          plhs[0] = sf_c2_flying_ip_1_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "3rwZHnUzbQAN5nKWURoC5E") == 0) {
          extern mxArray *sf_c3_flying_ip_1_updateBuildInfo_args_info(void);
          plhs[0] = sf_c3_flying_ip_1_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "ufK10B2QEcpQZqIQ1RwsF") == 0) {
          extern mxArray *sf_c4_flying_ip_1_updateBuildInfo_args_info(void);
          plhs[0] = sf_c4_flying_ip_1_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "xsI6r66MQduLdhBqKZoRaB") == 0) {
          extern mxArray *sf_c5_flying_ip_1_updateBuildInfo_args_info(void);
          plhs[0] = sf_c5_flying_ip_1_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void flying_ip_1_debug_initialize(struct SfDebugInstanceStruct* debugInstance)
{
  _flying_ip_1MachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "flying_ip_1","sfun",0,5,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,_flying_ip_1MachineNumber_,
    0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,_flying_ip_1MachineNumber_,
    0);
}

void flying_ip_1_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_flying_ip_1_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("flying_ip_1",
      "flying_ip_1");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_flying_ip_1_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
