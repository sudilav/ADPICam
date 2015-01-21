/* PICam.cpp */
#include "ADPICam.h"
#include <cstring>
#include <string>
#include <cmath>
#include <algorithm>
#include <epicsTime.h>
#include <epicsExit.h>

#define MAX_ENUM_STATES 16

static void piHandleNewImageTaskC(void *drvPvt);

/** Configuration command for PICAM driver; creates a new PICam object.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] maxBuffers The maximum number of NDArray buffers that the
 *            NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited number
 *            of buffers.
 * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for
 *            this driver is allowed to allocate. Set this to -1 to allow an
 *            unlimited amount of memory.
 * \param[in] priority The thread priority for the asyn port driver thread if
 *            ASYN_CANBLOCK is set in asynFlags.
 * \param[in] stackSize The stack size for the asyn port driver thread if
 *            ASYN_CANBLOCK is set in asynFlags.
 */
extern "C" int PICamConfig(const char *portName, int maxBuffers,
        size_t maxMemory, int priority, int stackSize) {
    new ADPICam(portName, maxBuffers, maxMemory, priority, stackSize);
    return (asynSuccess);
}

/** Configuration command for PICAM driver; creates a new PICam object.
 * \param[in]  demoCameraName String identifying demoCameraName
 */
extern "C" int PICamAddDemoCamera(const char *demoCameraName) {
    ADPICam::piAddDemoCamera(demoCameraName);
    return (asynSuccess);
}

/**
 * Callback function for exit hook
 */
static void exitCallbackC(void *pPvt){
    ADPICam *pADPICam = (ADPICam*)pPvt;
    delete pADPICam;
}

ADPICam * ADPICam::ADPICam_Instance = NULL;
const char *ADPICam::notAvailable = "N/A";
const char *ADPICam::driverName = "PICam";


/**
 *
 */
ADPICam::ADPICam(const char *portName, int maxBuffers, size_t maxMemory,
        int priority, int stackSize) :
        ADDriver(portName, 1, int(NUM_PICAM_PARAMS), maxBuffers, maxMemory,
        asynEnumMask, asynEnumMask, ASYN_CANBLOCK, 1, priority, stackSize) {
    int status = asynSuccess;
    static const char *functionName = "PICam";
    pibln libInitialized;
    PicamCameraID demoId;
    PicamError error = PicamError_None;
    const pichar *errorString;

    currentCameraHandle = NULL;
    selectedCameraIndex = -1;
    availableCamerasCount = 0;
    unavailableCamerasCount = 0;
    error = Picam_IsLibraryInitialized(&libInitialized);
    if (libInitialized) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s%s Found Picam Library initialized, unitializing\n", driverName,
            functionName);
        error = Picam_UninitializeLibrary();
        if (error != PicamError_None) {
            Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                    &errorString);
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s%s Trouble Uninitializing Picam Library: %s\n", driverName,
                    functionName, errorString);
            Picam_DestroyString(errorString);
            return;
        }
    }

    error = Picam_InitializeLibrary();
    if (error != PicamError_None) {
        //Try Again.
        error = Picam_InitializeLibrary();
        if (error != PicamError_None) {
            Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                    &errorString);
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s%s Trouble Initializing Picam Library: %s\n", driverName,
                    functionName, errorString);
            Picam_DestroyString(errorString);
            return;
        }
    }
    error = Picam_IsLibraryInitialized(&libInitialized);
    if (error != PicamError_None) {
        Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                &errorString);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s%s Trouble Checking if Picam Library is initialized: %s\n",
                driverName,
                functionName, errorString);
        Picam_DestroyString(errorString);
        
        return;
    }
    ADPICam_Instance = this;
    if (!libInitialized) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s%s Trouble Initializing Picam Library\n", driverName,
                functionName);
    }

    //Open First available camera.  If no camera is available,
    // then open a demo camera
    error = Picam_OpenFirstCamera(&currentCameraHandle);

    if (error != PicamError_None) {
        if (error == PicamError_NoCamerasAvailable) {
            error = Picam_ConnectDemoCamera(PicamModel_Quadro4320,
                    "CamNotFoundOnInit", &demoId);
            error = Picam_OpenFirstCamera(&currentCameraHandle);
        } else {
            Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                    &errorString);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s Error opening first camera: %s\n", driverName,
                    functionName, errorString);
            Picam_DestroyString(errorString);
        }
    }
    PicamAdvanced_GetCameraDevice(currentCameraHandle, &currentDeviceHandle);
    selectedCameraIndex = 0;
    lock();
    createParam(PICAM_VersionNumberString, asynParamOctet,
            &PICAM_VersionNumber);
    // Available Camera List
    createParam(PICAM_AvailableCamerasString, asynParamInt32,
            &PICAM_AvailableCameras);
    createParam(PICAM_CameraInterfaceString, asynParamOctet,
            &PICAM_CameraInterface);
    createParam(PICAM_SensorNameString, asynParamOctet, &PICAM_SensorName);
    createParam(PICAM_SerialNumberString, asynParamOctet, &PICAM_SerialNumber);
    createParam(PICAM_FirmwareRevisionString, asynParamOctet,
            &PICAM_FirmwareRevision);
    //Unavailable Camera List
    createParam(PICAM_UnavailableCamerasString, asynParamInt32,
            &PICAM_UnavailableCameras);
    createParam(PICAM_CameraInterfaceUnavailableString, asynParamOctet,
            &PICAM_CameraInterfaceUnavailable);
    createParam(PICAM_SensorNameUnavailableString, asynParamOctet,
            &PICAM_SensorNameUnavailable);
    createParam(PICAM_SerialNumberUnavailableString, asynParamOctet,
            &PICAM_SerialNumberUnavailable);
    createParam(PICAM_FirmwareRevisionUnavailableString, asynParamOctet,
            &PICAM_FirmwareRevisionUnavailable);
    //Shutter
    createParam(PICAM_ShutterDelayResolutionString , asynParamInt32,
            &PICAM_ShutterDelayResolution);
    createParam(PICAM_ShutterTimingModeString , asynParamInt32,
            &PICAM_ShutterTimingMode);

    //Intensifier
    createParam(PICAM_BracketGatingString , asynParamInt32,
            &PICAM_BracketGating);
    //TODO  CustomModulationSequence needs Modulation Type
    //TODO  DifEndingGate  needs pulse type
    //TODO  DifStartingGate needs pulse type
    createParam(PICAM_EMIccdGainString , asynParamInt32,
            &PICAM_EMIccdGain);
    createParam(PICAM_EMIccdGainControlModeString , asynParamInt32,
            &PICAM_EMIccdGainControlMode);
    createParam(PICAM_EnableIntensifierString , asynParamInt32,
            &PICAM_EnableIntensifier);
    createParam(PICAM_EnableModulationString , asynParamInt32,
            &PICAM_EnableModulation);
    createParam(PICAM_GatingModeString , asynParamInt32,
            &PICAM_GatingMode);
    createParam(PICAM_GatingSpeedString , asynParamOctet,
            &PICAM_GatingSpeed);
    createParam(PICAM_IntensifierDiameterString , asynParamFloat64,
            &PICAM_IntensifierDiameter);
    createParam(PICAM_IntensifierGainString , asynParamInt32,
            &PICAM_IntensifierGain);
    createParam(PICAM_IntensifierOptionsString , asynParamOctet,
            &PICAM_IntensifierOptions);
    createParam(PICAM_IntensifierStatusString , asynParamOctet,
            &PICAM_IntensifierStatus);
    createParam(PICAM_ModulationDurationString , asynParamFloat64,
            &PICAM_ModulationDuration);
    createParam(PICAM_ModulationFrequencyString , asynParamFloat64,
            &PICAM_ModulationFrequency);
    createParam(PICAM_PhosphorDecayDelayString , asynParamFloat64,
            &PICAM_PhosphorDecayDelay);
    createParam(PICAM_PhosphorDecayDelayResolutionString , asynParamInt32,
            &PICAM_PhosphorDecayDelayResolution);
    createParam(PICAM_PhosphorTypeString , asynParamOctet,
            &PICAM_PhosphorType);
    createParam(PICAM_PhotocathodeSensitivityString , asynParamOctet,
            &PICAM_PhotocathodeSensitivity);
    //TODO Repetitive Gate needs Pulse Type
    createParam(PICAM_RepetitiveModulationString , asynParamFloat64,
            &PICAM_RepetitiveModulation);
    createParam(PICAM_SequentialStartingModulationPhaseString , asynParamFloat64,
            &PICAM_SequentialStartingModulationPhase);
    createParam(PICAM_SequentialEndingModulationPhaseString , asynParamFloat64,
            &PICAM_SequentialEndingModulationPhase);
    //TODO SequentialEndingGate needs Pulse Type
    createParam(PICAM_SequentialGateStepCountString , asynParamInt32,
            &PICAM_SequentialGateStepCount);
    createParam(PICAM_SequentialGateStepIterationsString , asynParamInt32,
            &PICAM_SequentialGateStepIterations);
    //TODO SequentialEndingGate needs Pulse Type

    //Analog to Digital Conversion
    createParam(PICAM_AdcAnalogGainString, asynParamInt32,
            &PICAM_AdcAnalogGain);
    createParam(PICAM_AdcBitDepthString, asynParamInt32, &PICAM_AdcBitDepth);
    createParam(PICAM_AdcEMGainString, asynParamInt32, &PICAM_AdcEMGain);
    createParam(PICAM_AdcQualityString, asynParamInt32, &PICAM_AdcQuality);
    createParam(PICAM_AdcSpeedString, asynParamInt32, &PICAM_AdcSpeed);
    createParam(PICAM_CorrectPixelBiasString, asynParamInt32,
            &PICAM_CorrectPixelBias);

    // Hardware I/O
    //createParam(PICAM_Aux)  // Need to figure out pulse output
    createParam(PICAM_EnableModulationOutputSignalString, asynParamInt32,
            &PICAM_EnableModulationOutputSignal);
    createParam(PICAM_ModulationOutputSignalFrequencyString,
            asynParamFloat64,
            &PICAM_ModulationOutputSignalFrequency);
    createParam(PICAM_ModulationOutputSignalAmplitudeString,
            asynParamFloat64,
            &PICAM_ModulationOutputSignalAmplitude);
    createParam(PICAM_EnableSyncMasterString, asynParamInt32,
            &PICAM_EnableSyncMaster);
    createParam(PICAM_InvertOutputSignalString, asynParamInt32,
            &PICAM_InvertOutputSignal);
    createParam(PICAM_OutputSignalString, asynParamInt32,
            &PICAM_OutputSignal);
    createParam(PICAM_SyncMaster2DelayString, asynParamFloat64,
            &PICAM_SyncMaster2Delay);
    createParam(PICAM_TriggerCouplingString, asynParamInt32,
            &PICAM_TriggerCoupling);
    createParam(PICAM_TriggerDeterminationString, asynParamInt32,
            &PICAM_TriggerDetermination);
    createParam(PICAM_TriggerFrequencyString, asynParamFloat64,
            &PICAM_TriggerFrequency);
    //TriggerResponse is hooked up to ADTrigger
    createParam(PICAM_TriggerSourceString, asynParamInt32,
            &PICAM_TriggerSource);
    createParam(PICAM_TriggerTerminationString, asynParamInt32,
            &PICAM_TriggerTermination);
    createParam(PICAM_TriggerThresholdString, asynParamFloat64,
            &PICAM_TriggerThreshold);

    // Readout Control
    createParam(PICAM_AccumulationsString, asynParamInt32,
            &PICAM_Accumulations);
    createParam(PICAM_EnableNondestructiveReadoutString, asynParamInt32,
            &PICAM_EnableNondestructiveReadout);
    createParam(PICAM_KineticsWindowHeightString, asynParamInt32,
            &PICAM_KineticsWindowHeight);
    createParam(PICAM_NondestructiveReadoutPeriodString, asynParamFloat64,
            &PICAM_NondestructiveReadoutPeriod);
    createParam(PICAM_ReadoutControlModeString, asynParamInt32,
            &PICAM_ReadoutControlMode);
    createParam(PICAM_ReadoutOrientationString, asynParamOctet,
            &PICAM_ReadoutOrientation);
    createParam(PICAM_ReadoutPortCountString, asynParamInt32,
            &PICAM_ReadoutPortCount);
    createParam(PICAM_ReadoutTimeCalcString, asynParamFloat64,
            &PICAM_ReadoutTimeCalc);
    createParam(PICAM_VerticalShiftRateString, asynParamFloat64,
            &PICAM_VerticalShiftRate);

    // Data Acquisition
    createParam(PICAM_DisableDataFormattingString, asynParamInt32,
            &PICAM_DisableDataFormatting);
    createParam(PICAM_ExactReadoutCountMaxString, asynParamInt32,
            &PICAM_ExactReadoutCountMax);
    createParam(PICAM_FrameRateCalcString, asynParamFloat64,
            &PICAM_FrameRateCalc);
    createParam(PICAM_FramesPerReadoutString, asynParamInt32,
            &PICAM_FramesPerReadout);
    createParam(PICAM_FrameStrideString, asynParamInt32,
            &PICAM_FrameStride);
    createParam(PICAM_FrameTrackingBitDepthString, asynParamInt32,
            &PICAM_FrameTrackingBitDepth);
    createParam(PICAM_GateTrackingString, asynParamInt32,
            &PICAM_GateTracking);
    createParam(PICAM_GateTrackingBitDepthString, asynParamInt32,
            &PICAM_GateTrackingBitDepth);
    createParam(PICAM_ModulationTrackingString, asynParamInt32,
            &PICAM_ModulationTracking);
    createParam(PICAM_ModulationTrackingBitDepthString, asynParamInt32,
            &PICAM_ModulationTrackingBitDepth);
    createParam(PICAM_NormalizeOrientationString, asynParamInt32,
            &PICAM_NormalizeOrientation);
    createParam(PICAM_OnlineReadoutRateCalcString, asynParamFloat64,
            &PICAM_OnlineReadoutRateCalc);
    createParam(PICAM_OrientationString, asynParamOctet,
            &PICAM_Orientation);
    createParam(PICAM_PhotonDetectionModeString, asynParamInt32,
            &PICAM_PhotonDetectionMode);
    createParam(PICAM_PhotonDetectionThresholdString, asynParamFloat64,
            &PICAM_PhotonDetectionThreshold);
    createParam(PICAM_PixelBitDepthString, asynParamInt32,
            &PICAM_PixelBitDepth);
    createParam(PICAM_PixelFormatString, asynParamInt32,
            &PICAM_PixelFormat);
    createParam(PICAM_ReadoutCountString, asynParamInt32,
            &PICAM_ReadoutCount);
    createParam(PICAM_ReadoutRateCalcString, asynParamFloat64,
            &PICAM_ReadoutRateCalc);
    createParam(PICAM_ReadoutStrideString, asynParamInt32,
            &PICAM_ReadoutStride);
    createParam(PICAM_TimeStampBitDepthString, asynParamInt32,
            &PICAM_TimeStampBitDepth);
    createParam(PICAM_TimeStampResolutionString, asynParamInt32,
            &PICAM_TimeStampResolution);
    createParam(PICAM_TimeStampsString, asynParamInt32,
            &PICAM_TimeStamps);
    createParam(PICAM_TrackFramesString, asynParamInt32,
            &PICAM_TrackFrames);

    createParam(PICAM_CcdCharacteristicsString, asynParamOctet,
    		&PICAM_CcdCharacteristics);
    createParam(PICAM_PixelGapHeightString, asynParamFloat64,
    		&PICAM_PixelGapHeight);
    createParam(PICAM_PixelGapWidthString, asynParamFloat64,
    		&PICAM_PixelGapWidth);
    createParam(PICAM_PixelHeightString, asynParamFloat64,
    		&PICAM_PixelHeight);
    createParam(PICAM_PixelWidthString, asynParamFloat64,
    		&PICAM_PixelWidth);
    createParam(PICAM_SensorActiveBottomMarginString, asynParamInt32,
    		&PICAM_SensorActiveBottomMargin);
    createParam(PICAM_SensorActiveHeightString, asynParamInt32,
    		&PICAM_SensorActiveHeight);
    createParam(PICAM_SensorActiveLeftMarginString, asynParamInt32,
    		&PICAM_SensorActiveLeftMargin);
    createParam(PICAM_SensorActiveRightMarginString, asynParamInt32,
    		&PICAM_SensorActiveRightMargin);
    createParam(PICAM_SensorActiveTopMarginString, asynParamInt32,
    		&PICAM_SensorActiveTopMargin);
    createParam(PICAM_SensorActiveWidthString, asynParamInt32,
    		&PICAM_SensorActiveWidth);
    createParam(PICAM_SensorMaskedBottomMarginString, asynParamInt32,
    		&PICAM_SensorMaskedBottomMargin);
    createParam(PICAM_SensorMaskedHeightString, asynParamInt32,
    		&PICAM_SensorMaskedHeight);
    createParam(PICAM_SensorMaskedTopMarginString, asynParamInt32,
    		&PICAM_SensorMaskedTopMargin);
    createParam(PICAM_SensorSecondaryActiveHeightString, asynParamInt32,
    		&PICAM_SensorSecondaryActiveHeight);
    createParam(PICAM_SensorSecondaryMaskedHeightString, asynParamInt32,
    		&PICAM_SensorSecondaryMaskedHeight);
    createParam(PICAM_SensorTypeString, asynParamOctet,
    		&PICAM_SensorType);
    //Sensor Layout
    createParam(PICAM_ActiveBottomMarginString, asynParamInt32,
    		&PICAM_ActiveBottomMargin);
    createParam(PICAM_ActiveHeightString, asynParamInt32,
    		&PICAM_ActiveHeight);
    createParam(PICAM_ActiveLeftMarginString, asynParamInt32,
    		&PICAM_ActiveLeftMargin);
    createParam(PICAM_ActiveRightMarginString, asynParamInt32,
    		&PICAM_ActiveRightMargin);
    createParam(PICAM_ActiveTopMarginString, asynParamInt32,
    		&PICAM_ActiveTopMargin);
    createParam(PICAM_ActiveWidthString, asynParamInt32,
    		&PICAM_ActiveWidth);
    createParam(PICAM_MaskedBottomMarginString, asynParamInt32,
    		&PICAM_MaskedBottomMargin);
    createParam(PICAM_MaskedHeightString, asynParamInt32,
    		&PICAM_MaskedHeight);
    createParam(PICAM_MaskedTopMarginString, asynParamInt32,
    		&PICAM_MaskedTopMargin);
    createParam(PICAM_SecondaryActiveHeightString, asynParamInt32,
    		&PICAM_SecondaryActiveHeight);
    createParam(PICAM_SecondaryMaskedHeightString, asynParamInt32,
    		&PICAM_SecondaryMaskedHeight);
    //Sensor Cleaning
    createParam(PICAM_CleanBeforeExposureString, asynParamInt32,
    		&PICAM_CleanBeforeExposure);
    createParam(PICAM_CleanCycleCountString, asynParamInt32,
    		&PICAM_CleanCycleCount);
    createParam(PICAM_CleanCycleHeightString, asynParamInt32,
    		&PICAM_CleanCycleHeight);
    createParam(PICAM_CleanSectionFinalHeightString, asynParamInt32,
    		&PICAM_CleanSectionFinalHeight);
    createParam(PICAM_CleanSectionFinalHeightCountString, asynParamInt32,
    		&PICAM_CleanSectionFinalHeightCount);
    createParam(PICAM_CleanSerialRegisterString, asynParamInt32,
    		&PICAM_CleanSerialRegister);
    createParam(PICAM_CleanUntilTriggerString, asynParamInt32,
    		&PICAM_CleanUntilTrigger);

    //Sensor Temperature
    createParam(PICAM_DisableCoolingFanString, asynParamInt32,
            &PICAM_DisableCoolingFan);
    createParam(PICAM_EnableSensorWindowHeaterString, asynParamInt32,
            &PICAM_EnableSensorWindowHeater);
    createParam(PICAM_SensorTemperatureStatusString, asynParamOctet,
            &PICAM_SensorTemperatureStatus);

    // Display aids
    createParam(PICAM_EnableROIMinXInputString, asynParamInt32,
    		&PICAM_EnableROIMinXInput);
    createParam(PICAM_EnableROISizeXInputString, asynParamInt32,
    		&PICAM_EnableROISizeXInput);
    createParam(PICAM_EnableROIMinYInputString, asynParamInt32,
    		&PICAM_EnableROIMinYInput);
    createParam(PICAM_EnableROISizeYInputString, asynParamInt32,
    		&PICAM_EnableROISizeYInput);

    // Camera Parameter Exists for detector
    createParam(PICAM_ExposureTimeExistsString, asynParamInt32,
            &PICAM_ExposureTimeExists);
    createParam(PICAM_ShutterClosingDelayExistsString, asynParamInt32,
            &PICAM_ShutterClosingDelayExists);
    createParam(PICAM_ShutterDelayResolutionExistsString, asynParamInt32,
            &PICAM_ShutterDelayResolutionExists);
    createParam(PICAM_ShutterOpeningDelayExistsString, asynParamInt32,
            &PICAM_ShutterOpeningDelayExists);
    createParam(PICAM_ShutterTimingModeExistsString, asynParamInt32,
            &PICAM_ShutterTimingModeExists);
    createParam(PICAM_BracketGatingExistsString, asynParamInt32,
            &PICAM_BracketGatingExists);
    createParam(PICAM_CustomModulationSequenceExistsString, asynParamInt32,
            &PICAM_CustomModulationSequenceExists);
    createParam(PICAM_DifEndingGateExistsString, asynParamInt32,
            &PICAM_DifEndingGateExists);
    createParam(PICAM_DifStartingGateExistsString, asynParamInt32,
            &PICAM_DifStartingGateExists);
    createParam(PICAM_EMIccdGainExistsString, asynParamInt32,
            &PICAM_EMIccdGainExists);
    createParam(PICAM_EMIccdGainControlModeExistsString, asynParamInt32,
            &PICAM_EMIccdGainControlModeExists);
    createParam(PICAM_EnableIntensifierExistsString, asynParamInt32,
            &PICAM_EnableIntensifierExists);
    createParam(PICAM_EnableModulationExistsString, asynParamInt32,
            &PICAM_EnableModulationExists);
    createParam(PICAM_GatingModeExistsString, asynParamInt32,
            &PICAM_GatingModeExists);
    createParam(PICAM_GatingSpeedExistsString, asynParamInt32,
            &PICAM_GatingSpeedExists);
    createParam(PICAM_IntensifierDiameterExistsString, asynParamInt32,
            &PICAM_IntensifierDiameterExists);
    createParam(PICAM_IntensifierGainExistsString, asynParamInt32,
            &PICAM_IntensifierGainExists);
    createParam(PICAM_IntensifierOptionsExistsString, asynParamInt32,
            &PICAM_IntensifierOptionsExists);
    createParam(PICAM_IntensifierStatusExistsString, asynParamInt32,
            &PICAM_IntensifierStatusExists);
    createParam(PICAM_ModulationDurationExistsString, asynParamInt32,
            &PICAM_ModulationDurationExists);
    createParam(PICAM_ModulationFrequencyExistsString, asynParamInt32,
            &PICAM_ModulationFrequencyExists);
    createParam(PICAM_PhosphorDecayDelayExistsString, asynParamInt32,
            &PICAM_PhosphorDecayDelayExists);
    createParam(PICAM_PhosphorDecayDelayResolutionExistsString, asynParamInt32,
            &PICAM_PhosphorDecayDelayResolutionExists);
    createParam(PICAM_PhosphorTypeExistsString, asynParamInt32,
            &PICAM_PhosphorTypeExists);
    createParam(PICAM_PhotocathodeSensitivityExistsString, asynParamInt32,
            &PICAM_PhotocathodeSensitivityExists);
    createParam(PICAM_RepetitiveGateExistsString, asynParamInt32,
            &PICAM_RepetitiveGateExists);
    createParam(PICAM_RepetitiveModulationPhaseExistsString, asynParamInt32,
            &PICAM_RepetitiveModulationPhaseExists);
    createParam(PICAM_SequentialStartingModulationPhaseExistsString,
            asynParamInt32, &PICAM_SequentialStartingModulationPhaseExists);
    createParam(PICAM_SequentialEndingModulationPhaseExistsString,
    		asynParamInt32,
            &PICAM_SequentialEndingModulationPhaseExists);
    createParam(PICAM_SequentialEndingGateExistsString, asynParamInt32,
            &PICAM_SequentialEndingGateExists);
    createParam(PICAM_SequentialGateStepCountExistsString, asynParamInt32,
            &PICAM_SequentialGateStepCountExists);
    createParam(PICAM_SequentialGateStepIterationsExistsString, asynParamInt32,
            &PICAM_SequentialGateStepIterationsExists);
    createParam(PICAM_SequentialStartingGateExistsString, asynParamInt32,
            &PICAM_SequentialStartingGateExists);
    createParam(PICAM_AdcAnalogGainExistsString, asynParamInt32,
            &PICAM_AdcAnalogGainExists);
    createParam(PICAM_AdcBitDepthExistsString, asynParamInt32,
            &PICAM_AdcBitDepthExists);
    createParam(PICAM_AdcEMGainExistsString, asynParamInt32,
            &PICAM_AdcEMGainExists);
    createParam(PICAM_AdcQualityExistsString, asynParamInt32,
            &PICAM_AdcQualityExists);
    createParam(PICAM_AdcSpeedExistsString, asynParamInt32,
            &PICAM_AdcSpeedExists);
    createParam(PICAM_CorrectPixelBiasExistsString, asynParamInt32,
            &PICAM_CorrectPixelBiasExists);
    createParam(PICAM_AuxOutputExistsString, asynParamInt32,
            &PICAM_AuxOutputExists);
    createParam(PICAM_EnableModulationOutputSignalExistsString, asynParamInt32,
            &PICAM_EnableModulationOutputSignalExists);
    createParam(PICAM_ModulationOutputSignalFrequencyExistsString,
            asynParamInt32,
            &PICAM_EnableModulationOutputSignalFrequencyExists);
    createParam(PICAM_ModulationOutputSignalAmplitudeExistsString,
            asynParamInt32,
            &PICAM_EnableModulationOutputSignalAmplitudeExists);
    createParam(PICAM_EnableSyncMasterExistsString, asynParamInt32,
            &PICAM_EnableSyncMasterExists);
    createParam(PICAM_InvertOutputSignalExistsString, asynParamInt32,
            &PICAM_InvertOutputSignalExists);
    createParam(PICAM_OutputSignalExistsString, asynParamInt32,
            &PICAM_OutputSignalExists);
    createParam(PICAM_SyncMaster2DelayExistsString, asynParamInt32,
            &PICAM_SyncMaster2DelayExists);
    createParam(PICAM_TriggerCouplingExistsString, asynParamInt32,
            &PICAM_TriggerCouplingExists);
    createParam(PICAM_TriggerDeterminationExistsString, asynParamInt32,
            &PICAM_TriggerDeterminationExists);
    createParam(PICAM_TriggerFrequencyExistsString, asynParamInt32,
            &PICAM_TriggerFrequencyExists);
    createParam(PICAM_TriggerResponseExistsString, asynParamInt32,
            &PICAM_TriggerResponseExists);
    createParam(PICAM_TriggerSourceExistsString, asynParamInt32,
            &PICAM_TriggerSourceExists);
    createParam(PICAM_TriggerTerminationExistsString, asynParamInt32,
            &PICAM_TriggerTerminationExists);
    createParam(PICAM_TriggerThresholdExistsString, asynParamInt32,
            &PICAM_TriggerThresholdExists);
    createParam(PICAM_AccumulationsExistsString, asynParamInt32,
            &PICAM_AccumulationsExists);
    createParam(PICAM_EnableNondestructiveReadoutExistsString, asynParamInt32,
            &PICAM_EnableNondestructiveReadoutExists);
    createParam(PICAM_KineticsWindowHeightExistsString, asynParamInt32,
            &PICAM_KineticsWindowHeightExists);
    createParam(PICAM_NondestructiveReadoutPeriodExistsString, asynParamInt32,
            &PICAM_NondestructiveReadoutPeriodExists);
    createParam(PICAM_ReadoutControlModeExistsString, asynParamInt32,
            &PICAM_ReadoutControlModeExists);
    createParam(PICAM_ReadoutOrientationExistsString, asynParamInt32,
            &PICAM_ReadoutOrientationExists);
    createParam(PICAM_ReadoutPortCountExistsString, asynParamInt32,
            &PICAM_ReadoutPortCountExists);
    createParam(PICAM_ReadoutTimeCalculationExistsString, asynParamInt32,
            &PICAM_ReadoutTimeCalculationExists);
    createParam(PICAM_VerticalShiftRateExistsString, asynParamInt32,
            &PICAM_VerticalShiftRateExists);
    createParam(PICAM_DisableDataFormattingExistsString, asynParamInt32,
            &PICAM_DisableDataFormattingExists);
    createParam(PICAM_ExactReadoutCountMaximumExistsString, asynParamInt32,
            &PICAM_ExactReadoutCountMaximumExists);
    createParam(PICAM_FrameRateCalculationExistsString, asynParamInt32,
            &PICAM_FrameRateCalculationExists);
    createParam(PICAM_FrameSizeExistsString, asynParamInt32,
            &PICAM_FrameSizeExists);
    createParam(PICAM_FramesPerReadoutExistsString, asynParamInt32,
            &PICAM_FramesPerReadoutExists);
    createParam(PICAM_FrameStrideExistsString, asynParamInt32,
            &PICAM_FrameStrideExists);
    createParam(PICAM_FrameTrackingBitDepthExistsString, asynParamInt32,
            &PICAM_FrameTrackingBitDepthExists);
    createParam(PICAM_GateTrackingExistsString, asynParamInt32,
            &PICAM_GateTrackingExists);
    createParam(PICAM_GateTrackingBitDepthExistsString, asynParamInt32,
            &PICAM_GateTrackingBitDepthExists);
    createParam(PICAM_ModulationTrackingExistsString, asynParamInt32,
            &PICAM_ModulationTrackingExists);
    createParam(PICAM_ModulationTrackingBitDepthExistsString, asynParamInt32,
            &PICAM_ModulationTrackingBitDepthExists);
    createParam(PICAM_NormalizeOrientationExistsString, asynParamInt32,
            &PICAM_NormalizeOrientationExists);
    createParam(PICAM_OnlineReadoutRateCalculationExistsString, asynParamInt32,
            &PICAM_OnlineReadoutRateCalculationExists);
    createParam(PICAM_OrientationExistsString, asynParamInt32,
            &PICAM_OrientationExists);
    createParam(PICAM_PhotonDetectionModeExistsString, asynParamInt32,
            &PICAM_PhotonDetectionModeExists);
    createParam(PICAM_PhotonDetectionThresholdExistsString, asynParamInt32,
            &PICAM_PhotonDetectionThresholdExists);
    createParam(PICAM_PixelBitDepthExistsString, asynParamInt32,
            &PICAM_PixelBitDepthExists);
    createParam(PICAM_PixelFormatExistsString, asynParamInt32,
            &PICAM_PixelFormatExists);
    createParam(PICAM_ReadoutCountExistsString, asynParamInt32,
            &PICAM_ReadoutCountExists);
    createParam(PICAM_ReadoutRateCalculationExistsString, asynParamInt32,
            &PICAM_ReadoutRateCalculationExists);
    createParam(PICAM_ReadoutStrideExistsString, asynParamInt32,
            &PICAM_ReadoutStrideExists);
    createParam(PICAM_RoisExistsString, asynParamInt32, &PICAM_RoisExists);
    createParam(PICAM_TimeStampBitDepthExistsString, asynParamInt32,
            &PICAM_TimeStampBitDepthExists);
    createParam(PICAM_TimeStampResolutionExistsString, asynParamInt32,
            &PICAM_TimeStampResolutionExists);
    createParam(PICAM_TimeStampsExistsString, asynParamInt32,
            &PICAM_TimeStampsExists);
    createParam(PICAM_TrackFramesExistsString, asynParamInt32,
            &PICAM_TrackFramesExists);
    createParam(PICAM_CcdCharacteristicsExistsString, asynParamInt32,
            &PICAM_CcdCharacteristicsExists);
    createParam(PICAM_PixelGapHeightExistsString, asynParamInt32,
            &PICAM_PixelGapHeightExists);
    createParam(PICAM_PixelGapWidthExistsString, asynParamInt32,
            &PICAM_PixelGapWidthExists);
    createParam(PICAM_PixelHeightExistsString, asynParamInt32,
            &PICAM_PixelHeightExists);
    createParam(PICAM_PixelWidthExistsString, asynParamInt32,
            &PICAM_PixelWidthExists);
    createParam(PICAM_SensorActiveBottomMarginExistsString, asynParamInt32,
            &PICAM_SensorActiveBottomMarginExists);
    createParam(PICAM_SensorActiveHeightExistsString, asynParamInt32,
            &PICAM_SensorActiveHeightExists);
    createParam(PICAM_SensorActiveLeftMarginExistsString, asynParamInt32,
            &PICAM_SensorActiveLeftMarginExists);
    createParam(PICAM_SensorActiveRightMarginExistsString, asynParamInt32,
            &PICAM_SensorActiveRightMarginExists);
    createParam(PICAM_SensorActiveTopMarginExistsString, asynParamInt32,
            &PICAM_SensorActiveTopMarginExists);
    createParam(PICAM_SensorActiveWidthExistsString, asynParamInt32,
            &PICAM_SensorActiveWidthExists);
    createParam(PICAM_SensorMaskedBottomMarginExistsString, asynParamInt32,
            &PICAM_SensorMaskedBottomMarginExists);
    createParam(PICAM_SensorMaskedHeightExistsString, asynParamInt32,
            &PICAM_SensorMaskedHeightExists);
    createParam(PICAM_SensorMaskedTopMarginExistsString, asynParamInt32,
            &PICAM_SensorMaskedTopMarginExists);
    createParam(PICAM_SensorSecondaryActiveHeightExistsString, asynParamInt32,
            &PICAM_SensorSecondaryActiveHeightExists);
    createParam(PICAM_SensorSecondaryMaskedHeightExistsString, asynParamInt32,
            &PICAM_SensorSecondaryMaskedHeightExists);
    createParam(PICAM_SensorTypeExistsString, asynParamInt32,
            &PICAM_SensorTypeExists);
    createParam(PICAM_ActiveBottomMarginExistsString, asynParamInt32,
            &PICAM_ActiveBottomMarginExists);
    createParam(PICAM_ActiveHeightExistsString, asynParamInt32,
            &PICAM_ActiveHeightExists);
    createParam(PICAM_ActiveLeftMarginExistsString, asynParamInt32,
            &PICAM_ActiveLeftMarginExists);
    createParam(PICAM_ActiveRightMarginExistsString, asynParamInt32,
            &PICAM_ActiveRightMarginExists);
    createParam(PICAM_ActiveTopMarginExistsString, asynParamInt32,
            &PICAM_ActiveTopMarginExists);
    createParam(PICAM_ActiveWidthExistsString, asynParamInt32,
            &PICAM_ActiveWidthExists);
    createParam(PICAM_MaskedBottomMarginExistsString, asynParamInt32,
            &PICAM_MaskedBottomMarginExists);
    createParam(PICAM_MaskedHeightExistsString, asynParamInt32,
            &PICAM_MaskedHeightExists);
    createParam(PICAM_MaskedTopMarginExistsString, asynParamInt32,
            &PICAM_MaskedTopMarginExists);
    createParam(PICAM_SecondaryActiveHeightExistsString, asynParamInt32,
            &PICAM_SecondaryActiveHeightExists);
    createParam(PICAM_SecondaryMaskedHeightExistsString, asynParamInt32,
            &PICAM_SecondaryMaskedHeightExists);
    createParam(PICAM_CleanBeforeExposureExistsString, asynParamInt32,
            &PICAM_CleanBeforeExposureExists);
    createParam(PICAM_CleanCycleCountExistsString, asynParamInt32,
            &PICAM_CleanCycleCountExists);
    createParam(PICAM_CleanCycleHeightExistsString, asynParamInt32,
            &PICAM_CleanCycleHeightExists);
    createParam(PICAM_CleanSectionFinalHeightExistsString, asynParamInt32,
            &PICAM_CleanSectionFinalHeightExists);
    createParam(PICAM_CleanSectionFinalHeightCountExistsString, asynParamInt32,
            &PICAM_CleanSectionFinalHeightCountExists);
    createParam(PICAM_CleanSerialRegisterExistsString, asynParamInt32,
            &PICAM_CleanSerialRegisterExists);
    createParam(PICAM_CleanUntilTriggerExistsString, asynParamInt32,
            &PICAM_CleanUntilTriggerExists);
    createParam(PICAM_DisableCoolingFanExistsString, asynParamInt32,
            &PICAM_DisableCoolingFanExists);
    createParam(PICAM_EnableSensorWindowHeaterExistsString, asynParamInt32,
            &PICAM_EnableSensorWindowHeaterExists);
    createParam(PICAM_SensorTemperatureReadingExistsString, asynParamInt32,
            &PICAM_SensorTemperatureReadingExists);
    createParam(PICAM_SensorTemperatureSetPointExistsString, asynParamInt32,
            &PICAM_SensorTemperatureSetPointExists);
    createParam(PICAM_SensorTemperatureStatusExistsString, asynParamInt32,
            &PICAM_SensorTemperatureStatusExists);
    // Camera Parameter Relevance
    createParam(PICAM_ExposureTimeRelString, asynParamInt32,
            &PICAM_ExposureTimeRelevant);
    createParam(PICAM_ShutterClosingDelayRelString, asynParamInt32,
            &PICAM_ShutterClosingDelayRelevant);
    createParam(PICAM_ShutterDelayResolutionRelString, asynParamInt32,
            &PICAM_ShutterDelayResolutionRelevant);
    createParam(PICAM_ShutterOpeningDelayRelString, asynParamInt32,
            &PICAM_ShutterOpeningDelayRelevant);
    createParam(PICAM_ShutterTimingModeRelString, asynParamInt32,
            &PICAM_ShutterTimingModeRelevant);
    createParam(PICAM_BracketGatingRelString, asynParamInt32,
            &PICAM_BracketGatingRelevant);
    createParam(PICAM_CustomModulationSequenceRelString, asynParamInt32,
            &PICAM_CustomModulationSequenceRelevant);
    createParam(PICAM_DifEndingGateRelString, asynParamInt32,
            &PICAM_DifEndingGateRelevant);
    createParam(PICAM_DifStartingGateRelString, asynParamInt32,
            &PICAM_DifStartingGateRelevant);
    createParam(PICAM_EMIccdGainRelString, asynParamInt32,
            &PICAM_EMIccdGainRelevant);
    createParam(PICAM_EMIccdGainControlModeRelString, asynParamInt32,
            &PICAM_EMIccdGainControlModeRelevant);
    createParam(PICAM_EnableIntensifierRelString, asynParamInt32,
            &PICAM_EnableIntensifierRelevant);
    createParam(PICAM_EnableModulationRelString, asynParamInt32,
            &PICAM_EnableModulationRelevant);
    createParam(PICAM_GatingModeRelString, asynParamInt32,
            &PICAM_GatingModeRelevant);
    createParam(PICAM_GatingSpeedRelString, asynParamInt32,
            &PICAM_GatingSpeedRelevant);
    createParam(PICAM_IntensifierDiameterRelString, asynParamInt32,
            &PICAM_IntensifierDiameterRelevant);
    createParam(PICAM_IntensifierGainRelString, asynParamInt32,
            &PICAM_IntensifierGainRelevant);
    createParam(PICAM_IntensifierOptionsRelString, asynParamInt32,
            &PICAM_IntensifierOptionsRelevant);
    createParam(PICAM_IntensifierStatusRelString, asynParamInt32,
            &PICAM_IntensifierStatusRelevant);
    createParam(PICAM_ModulationDurationRelString, asynParamInt32,
            &PICAM_ModulationDurationRelevant);
    createParam(PICAM_ModulationFrequencyRelString, asynParamInt32,
            &PICAM_ModulationFrequencyRelevant);
    createParam(PICAM_PhosphorDecayDelayRelString, asynParamInt32,
            &PICAM_PhosphorDecayDelayRelevant);
    createParam(PICAM_PhosphorDecayDelayResolutionRelString, asynParamInt32,
            &PICAM_PhosphorDecayDelayResolutionRelevant);
    createParam(PICAM_PhosphorTypeRelString, asynParamInt32,
            &PICAM_PhosphorTypeRelevant);
    createParam(PICAM_PhotocathodeSensitivityRelString, asynParamInt32,
            &PICAM_PhotocathodeSensitivityRelevant);
    createParam(PICAM_RepetitiveGateRelString, asynParamInt32,
            &PICAM_RepetitiveGateRelevant);
    createParam(PICAM_RepetitiveModulationPhaseRelString, asynParamInt32,
            &PICAM_RepetitiveModulationPhaseRelevant);
    createParam(PICAM_SequentialStartingModulationPhaseRelString,
            asynParamInt32, &PICAM_SequentialStartingModulationPhaseRelevant);
    createParam(PICAM_SequentialEndingModulationPhaseRelString, asynParamInt32,
            &PICAM_SequentialEndingModulationPhaseRelevant);
    createParam(PICAM_SequentialEndingGateRelString, asynParamInt32,
            &PICAM_SequentialEndingGateRelevant);
    createParam(PICAM_SequentialGateStepCountRelString, asynParamInt32,
            &PICAM_SequentialGateStepCountRelevant);
    createParam(PICAM_SequentialGateStepIterationsRelString, asynParamInt32,
            &PICAM_SequentialGateStepIterationsRelevant);
    createParam(PICAM_SequentialStartingGateRelString, asynParamInt32,
            &PICAM_SequentialStartingGateRelevant);
    createParam(PICAM_AdcAnalogGainRelString, asynParamInt32,
            &PICAM_AdcAnalogGainRelevant);
    createParam(PICAM_AdcBitDepthRelString, asynParamInt32,
            &PICAM_AdcBitDepthRelevant);
    createParam(PICAM_AdcEMGainRelString, asynParamInt32,
            &PICAM_AdcEMGainRelevant);
    createParam(PICAM_AdcQualityRelString, asynParamInt32,
            &PICAM_AdcQualityRelevant);
    createParam(PICAM_AdcSpeedRelString, asynParamInt32,
            &PICAM_AdcSpeedRelevant);
    createParam(PICAM_CorrectPixelBiasRelString, asynParamInt32,
            &PICAM_CorrectPixelBiasRelevant);
    createParam(PICAM_AuxOutputRelString, asynParamInt32,
            &PICAM_AuxOutputRelevant);
    createParam(PICAM_EnableModulationOutputSignalRelString, asynParamInt32,
            &PICAM_EnableModulationOutputSignalRelevant);
    createParam(PICAM_ModulationOutputSignalFrequencyRelString,
            asynParamInt32,
            &PICAM_EnableModulationOutputSignalFrequencyRelevant);
    createParam(PICAM_ModulationOutputSignalAmplitudeRelString,
            asynParamInt32,
            &PICAM_EnableModulationOutputSignalAmplitudeRelevant);
    createParam(PICAM_EnableSyncMasterRelString, asynParamInt32,
            &PICAM_EnableSyncMasterRelevant);
    createParam(PICAM_InvertOutputSignalRelString, asynParamInt32,
            &PICAM_InvertOutputSignalRelevant);
    createParam(PICAM_OutputSignalRelString, asynParamInt32,
            &PICAM_OutputSignalRelevant);
    createParam(PICAM_SyncMaster2DelayRelString, asynParamInt32,
            &PICAM_SyncMaster2DelayRelevant);
    createParam(PICAM_TriggerCouplingRelString, asynParamInt32,
            &PICAM_TriggerCouplingRelevant);
    createParam(PICAM_TriggerDeterminationRelString, asynParamInt32,
            &PICAM_TriggerDeterminationRelevant);
    createParam(PICAM_TriggerFrequencyRelString, asynParamInt32,
            &PICAM_TriggerFrequencyRelevant);
    createParam(PICAM_TriggerResponseRelString, asynParamInt32,
            &PICAM_TriggerResponseRelevant);
    createParam(PICAM_TriggerSourceRelString, asynParamInt32,
            &PICAM_TriggerSourceRelevant);
    createParam(PICAM_TriggerTerminationRelString, asynParamInt32,
            &PICAM_TriggerTerminationRelevant);
    createParam(PICAM_TriggerThresholdRelString, asynParamInt32,
            &PICAM_TriggerThresholdRelevant);
    createParam(PICAM_AccumulationsRelString, asynParamInt32,
            &PICAM_AccumulationsRelevant);
    createParam(PICAM_EnableNondestructiveReadoutRelString, asynParamInt32,
            &PICAM_EnableNondestructiveReadoutRelevant);
    createParam(PICAM_KineticsWindowHeightRelString, asynParamInt32,
            &PICAM_KineticsWindowHeightRelevant);
    createParam(PICAM_NondestructiveReadoutPeriodRelString, asynParamInt32,
            &PICAM_NondestructiveReadoutPeriodRelevant);
    createParam(PICAM_ReadoutControlModeRelString, asynParamInt32,
            &PICAM_ReadoutControlModeRelevant);
    createParam(PICAM_ReadoutOrientationRelString, asynParamInt32,
            &PICAM_ReadoutOrientationRelevant);
    createParam(PICAM_ReadoutPortCountRelString, asynParamInt32,
            &PICAM_ReadoutPortCountRelevant);
    createParam(PICAM_ReadoutTimeCalculationRelString, asynParamInt32,
            &PICAM_ReadoutTimeCalculationRelevant);
    createParam(PICAM_VerticalShiftRateRelString, asynParamInt32,
            &PICAM_VerticalShiftRateRelevant);
    createParam(PICAM_DisableDataFormattingRelString, asynParamInt32,
            &PICAM_DisableDataFormattingRelevant);
    createParam(PICAM_ExactReadoutCountMaximumRelString, asynParamInt32,
            &PICAM_ExactReadoutCountMaximumRelevant);
    createParam(PICAM_FrameRateCalculationRelString, asynParamInt32,
            &PICAM_FrameRateCalculationRelevant);
    createParam(PICAM_FrameSizeRelString, asynParamInt32,
            &PICAM_FrameSizeRelevant);
    createParam(PICAM_FramesPerReadoutRelString, asynParamInt32,
            &PICAM_FramesPerReadoutRelevant);
    createParam(PICAM_FrameStrideRelString, asynParamInt32,
            &PICAM_FrameStrideRelevant);
    createParam(PICAM_FrameTrackingBitDepthRelString, asynParamInt32,
            &PICAM_FrameTrackingBitDepthRelevant);
    createParam(PICAM_GateTrackingRelString, asynParamInt32,
            &PICAM_GateTrackingRelevant);
    createParam(PICAM_GateTrackingBitDepthRelString, asynParamInt32,
            &PICAM_GateTrackingBitDepthRelevant);
    createParam(PICAM_ModulationTrackingRelString, asynParamInt32,
            &PICAM_ModulationTrackingRelevant);
    createParam(PICAM_ModulationTrackingBitDepthRelString, asynParamInt32,
            &PICAM_ModulationTrackingBitDepthRelevant);
    createParam(PICAM_NormalizeOrientationRelString, asynParamInt32,
            &PICAM_NormalizeOrientationRelevant);
    createParam(PICAM_OnlineReadoutRateCalculationRelString, asynParamInt32,
            &PICAM_OnlineReadoutRateCalculationRelevant);
    createParam(PICAM_OrientationRelString, asynParamInt32,
            &PICAM_OrientationRelevant);
    createParam(PICAM_PhotonDetectionModeRelString, asynParamInt32,
            &PICAM_PhotonDetectionModeRelevant);
    createParam(PICAM_PhotonDetectionThresholdRelString, asynParamInt32,
            &PICAM_PhotonDetectionThresholdRelevant);
    createParam(PICAM_PixelBitDepthRelString, asynParamInt32,
            &PICAM_PixelBitDepthRelevant);
    createParam(PICAM_PixelFormatRelString, asynParamInt32,
            &PICAM_PixelFormatRelevant);
    createParam(PICAM_ReadoutCountRelString, asynParamInt32,
            &PICAM_ReadoutCountRelevant);
    createParam(PICAM_ReadoutRateCalculationRelString, asynParamInt32,
            &PICAM_ReadoutRateCalculationRelevant);
    createParam(PICAM_ReadoutStrideRelString, asynParamInt32,
            &PICAM_ReadoutStrideRelevant);
    createParam(PICAM_RoisRelString, asynParamInt32, &PICAM_RoisRelevant);
    createParam(PICAM_TimeStampBitDepthRelString, asynParamInt32,
            &PICAM_TimeStampBitDepthRelevant);
    createParam(PICAM_TimeStampResolutionRelString, asynParamInt32,
            &PICAM_TimeStampResolutionRelevant);
    createParam(PICAM_TimeStampsRelString, asynParamInt32,
            &PICAM_TimeStampsRelevant);
    createParam(PICAM_TrackFramesRelString, asynParamInt32,
            &PICAM_TrackFramesRelevant);
    createParam(PICAM_CcdCharacteristicsRelString, asynParamInt32,
            &PICAM_CcdCharacteristicsRelevant);
    createParam(PICAM_PixelGapHeightRelString, asynParamInt32,
            &PICAM_PixelGapHeightRelevant);
    createParam(PICAM_PixelGapWidthRelString, asynParamInt32,
            &PICAM_PixelGapWidthRelevant);
    createParam(PICAM_PixelHeightRelString, asynParamInt32,
            &PICAM_PixelHeightRelevant);
    createParam(PICAM_PixelWidthRelString, asynParamInt32,
            &PICAM_PixelWidthRelevant);
    createParam(PICAM_SensorActiveBottomMarginRelString, asynParamInt32,
            &PICAM_SensorActiveBottomMarginRelevant);
    createParam(PICAM_SensorActiveHeightRelString, asynParamInt32,
            &PICAM_SensorActiveHeightRelevant);
    createParam(PICAM_SensorActiveLeftMarginRelString, asynParamInt32,
            &PICAM_SensorActiveLeftMarginRelevant);
    createParam(PICAM_SensorActiveRightMarginRelString, asynParamInt32,
            &PICAM_SensorActiveRightMarginRelevant);
    createParam(PICAM_SensorActiveTopMarginRelString, asynParamInt32,
            &PICAM_SensorActiveTopMarginRelevant);
    createParam(PICAM_SensorActiveWidthRelString, asynParamInt32,
            &PICAM_SensorActiveWidthRelevant);
    createParam(PICAM_SensorMaskedBottomMarginRelString, asynParamInt32,
            &PICAM_SensorMaskedBottomMarginRelevant);
    createParam(PICAM_SensorMaskedHeightRelString, asynParamInt32,
            &PICAM_SensorMaskedHeightRelevant);
    createParam(PICAM_SensorMaskedTopMarginRelString, asynParamInt32,
            &PICAM_SensorMaskedTopMarginRelevant);
    createParam(PICAM_SensorSecondaryActiveHeightRelString, asynParamInt32,
            &PICAM_SensorSecondaryActiveHeightRelevant);
    createParam(PICAM_SensorSecondaryMaskedHeightRelString, asynParamInt32,
            &PICAM_SensorSecondaryMaskedHeightRelevant);
    createParam(PICAM_SensorTypeRelString, asynParamInt32,
            &PICAM_SensorTypeRelevant);
    createParam(PICAM_ActiveBottomMarginRelString, asynParamInt32,
            &PICAM_ActiveBottomMarginRelevant);
    createParam(PICAM_ActiveHeightRelString, asynParamInt32,
            &PICAM_ActiveHeightRelevant);
    createParam(PICAM_ActiveLeftMarginRelString, asynParamInt32,
            &PICAM_ActiveLeftMarginRelevant);
    createParam(PICAM_ActiveRightMarginRelString, asynParamInt32,
            &PICAM_ActiveRightMarginRelevant);
    createParam(PICAM_ActiveTopMarginRelString, asynParamInt32,
            &PICAM_ActiveTopMarginRelevant);
    createParam(PICAM_ActiveWidthRelString, asynParamInt32,
            &PICAM_ActiveWidthRelevant);
    createParam(PICAM_MaskedBottomMarginRelString, asynParamInt32,
            &PICAM_MaskedBottomMarginRelevant);
    createParam(PICAM_MaskedHeightRelString, asynParamInt32,
            &PICAM_MaskedHeightRelevant);
    createParam(PICAM_MaskedTopMarginRelString, asynParamInt32,
            &PICAM_MaskedTopMarginRelevant);
    createParam(PICAM_SecondaryActiveHeightRelString, asynParamInt32,
            &PICAM_SecondaryActiveHeightRelevant);
    createParam(PICAM_SecondaryMaskedHeightRelString, asynParamInt32,
            &PICAM_SecondaryMaskedHeightRelevant);
    createParam(PICAM_CleanBeforeExposureRelString, asynParamInt32,
            &PICAM_CleanBeforeExposureRelevant);
    createParam(PICAM_CleanCycleCountRelString, asynParamInt32,
            &PICAM_CleanCycleCountRelevant);
    createParam(PICAM_CleanCycleHeightRelString, asynParamInt32,
            &PICAM_CleanCycleHeightRelevant);
    createParam(PICAM_CleanSectionFinalHeightRelString, asynParamInt32,
            &PICAM_CleanSectionFinalHeightRelevant);
    createParam(PICAM_CleanSectionFinalHeightCountRelString, asynParamInt32,
            &PICAM_CleanSectionFinalHeightCountRelevant);
    createParam(PICAM_CleanSerialRegisterRelString, asynParamInt32,
            &PICAM_CleanSerialRegisterRelevant);
    createParam(PICAM_CleanUntilTriggerRelString, asynParamInt32,
            &PICAM_CleanUntilTriggerRelevant);
    createParam(PICAM_DisableCoolingFanRelString, asynParamInt32,
            &PICAM_DisableCoolingFanRelevant);
    createParam(PICAM_EnableSensorWindowHeaterRelString, asynParamInt32,
            &PICAM_EnableSensorWindowHeaterRelevant);
    createParam(PICAM_SensorTemperatureReadingRelString, asynParamInt32,
            &PICAM_SensorTemperatureReadingRelevant);
    createParam(PICAM_SensorTemperatureSetPointRelString, asynParamInt32,
            &PICAM_SensorTemperatureSetPointRelevant);
    createParam(PICAM_SensorTemperatureStatusRelString, asynParamInt32,
            &PICAM_SensorTemperatureStatusRelevant);
    status = setStringParam(ADManufacturer, "Princeton Instruments");
    status |= setStringParam(ADModel, "Not Connected");
    status |= setIntegerParam(NDArraySize, 0);
    status |= setIntegerParam(NDDataType, NDUInt16);

    status |= setStringParam(PICAM_VersionNumber, "NOT DETERMINED1");
    status |= setIntegerParam(PICAM_AvailableCameras, 0);
    status |= setStringParam(PICAM_CameraInterface, "NOT DETERMINED2");
    status |= setStringParam(PICAM_SensorName, "NOT DETERMINED3");
    status |= setStringParam(PICAM_SerialNumber, "NOT DETERMINED4");
    status |= setStringParam(PICAM_FirmwareRevision, "NOT DETERMINED5");
    status |= setIntegerParam(PICAM_UnavailableCameras, 0);
    status |= setStringParam(PICAM_CameraInterfaceUnavailable,
            "NOT DETERMINED2");
    status |= setStringParam(PICAM_SensorNameUnavailable, "NOT DETERMINED3");
    status |= setStringParam(PICAM_SerialNumberUnavailable, "NOT DETERMINED4");
    status |= setStringParam(PICAM_FirmwareRevisionUnavailable,
            "NOT DETERMINED5");
    status |= setIntegerParam(ADNumImagesCounter, 1);
    status |= setIntegerParam(PICAM_EnableROIMinXInput, 1);
    status |= setIntegerParam(PICAM_EnableROISizeXInput, 1);
    status |= setIntegerParam(PICAM_EnableROIMinYInput, 1);
    status |= setIntegerParam(PICAM_EnableROISizeYInput, 1);
    callParamCallbacks();
    unlock();

    piLoadAvailableCameraIDs();
    setIntegerParam(PICAM_AvailableCameras, 0);
    callParamCallbacks();

    // Comment out for now.  This is making the IOC crash.  Need to move on to
    // reading parameters.
    piLoadUnavailableCameraIDs();
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to set camera parameters\n", driverName,
                functionName);
        return;
    }
    piHandleNewImageEvent = epicsEventCreate(epicsEventEmpty);
    /* Create the thread that updates the images */
    status = (epicsThreadCreate("piHandleNewImageTaskC",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)piHandleNewImageTaskC,
                                this) == NULL);
    initializeDetector();

    epicsAtExit(exitCallbackC, this);
}

/**
 * Destructor function.  Clear out Picam Library
 */
ADPICam::~ADPICam() {
    const char * functionName = "~ADPICam";
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s Enter\n",
            driverName,
            functionName);


    Picam_StopAcquisition(currentCameraHandle);
    PicamAdvanced_UnregisterForAcquisitionUpdated(currentDeviceHandle,
                    piAcquistionUpdated);
    piUnregisterRelevantWatch(currentCameraHandle);
    piUnregisterValueChangeWatch(currentCameraHandle);
    Picam_UninitializeLibrary();
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s Exit\n",
            driverName,
            functionName);
}

/**
 * Initialize Picam property initialization
 * - Read PICAM Library version
 * - Register camera discovery callback
 * - Start camera discovery
 */
asynStatus ADPICam::initializeDetector() {
    piint versionMajor, versionMinor, versionDistribution, versionReleased;

    static const char* functionName = "initializeDetector";
    const char *errorString;
    bool retVal = true;
    char picamVersion[16];
    int status = asynSuccess;
    PicamError error;

    // Read PICAM Library version #
    Picam_GetVersion(&versionMajor, &versionMinor, &versionDistribution,
            &versionReleased);
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s Initialized PICam Version %d.%d.%d.%d\n", driverName,
            functionName, versionMajor, versionMinor, versionDistribution,
            versionReleased);
    sprintf(picamVersion, "%d.%d.%d.%d", versionMajor, versionMinor,
            versionDistribution, versionReleased);
    status |= setStringParam(PICAM_VersionNumber, (const char *) picamVersion);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to set camera parameters\n", driverName,
                functionName);
        return asynError;
    }
    // Register callback for Camera Discovery
    error = PicamAdvanced_RegisterForDiscovery(piCameraDiscovered);
    if (error != PicamError_None) {

        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Fail to Register for Camera Discovery :%s\n", driverName,
                functionName, errorString);
        return asynError;
    }
    // Launch camera discovery process
    error = PicamAdvanced_DiscoverCameras();
    if (error != PicamError_None) {

        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Fail to start camera discovery :%s\n", driverName,
                functionName, errorString);
        return asynError;
    }

    return (asynStatus) status;
}

/**
 * Override method from asynPortDriver to populate pull-down lists
 * for camera parameters.  Note that for this to work asynEnumMask must be set
 * when calling ADDriver constructor
 */
asynStatus ADPICam::readEnum(asynUser *pasynUser, char *strings[], int values[],
        int severities[], size_t nElements, size_t *nIn) {
    static const char *functionName = "readEnum";
    int status = asynSuccess;
    PicamError error;
    char enumString[64];
    const char *modelString;
    const char *errorString;
    const char *parameterName;
    const char *NAString = "N.A. 0";
    PicamParameter picamParameter;
    int function = pasynUser->reason;

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "*******%s:%s: entry\n",
            driverName, functionName);

    *nIn = 0;
    if (function == PICAM_AvailableCameras) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "Getting Available IDs\n");
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s availableCamerasCount %d\n", driverName, functionName,
                availableCamerasCount);
        for (int ii = 0; ii < availableCamerasCount; ii++) {
            Picam_GetEnumerationString(PicamEnumeratedType_Model,
                    (piint) availableCameraIDs[ii].model, &modelString);
            pibln camConnected = false;
            Picam_IsCameraIDConnected(availableCameraIDs, &camConnected);
            sprintf(enumString, "%s", modelString);
            asynPrint(pasynUser, ASYN_TRACE_FLOW,
                    "\n%s:%s: \nCamera[%d]\n---%s\n---%d\n---%s\n---%s\n",
                    driverName, functionName, ii, modelString,
                    availableCameraIDs[ii].computer_interface,
                    availableCameraIDs[ii].sensor_name,
                    availableCameraIDs[ii].serial_number);
            if (strings[*nIn])
                free(strings[*nIn]);

            Picam_DestroyString(modelString);
            strings[*nIn] = epicsStrDup(enumString);
            values[*nIn] = ii;
            severities[*nIn] = 0;
            (*nIn)++;
        }

    } else if (function == PICAM_UnavailableCameras) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "Getting Unavailable IDs\n");
        for (int ii = 0; ii < unavailableCamerasCount; ii++) {
            Picam_GetEnumerationString(PicamEnumeratedType_Model,
                    (piint) unavailableCameraIDs[ii].model, &modelString);
            sprintf(enumString, "%s", modelString);
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "\n%s:%s: \nCamera[%d]\n---%s\n---%d\n---%s\n---%s\n",
                    driverName, functionName, ii, modelString,
                    unavailableCameraIDs[ii].computer_interface,
                    unavailableCameraIDs[ii].sensor_name,
                    unavailableCameraIDs[ii].serial_number);
            if (strings[*nIn])
                free(strings[*nIn]);

            Picam_DestroyString(modelString);
            strings[*nIn] = epicsStrDup(enumString);
            values[*nIn] = ii;
            severities[*nIn] = 0;
            (*nIn)++;
        }
    } else if ( piLookupPICamParameter(function, picamParameter) ==
            PicamError_None) {
        const PicamCollectionConstraint *constraints;
        const char *paramConstraintString;
        if (currentCameraHandle != NULL){
        	pibln paramExists;
        	Picam_DoesParameterExist(currentCameraHandle,
        			picamParameter,
					&paramExists);
        	if (paramExists){
				pibln isRelevant;
				error = Picam_IsParameterRelevant(currentCameraHandle,
						picamParameter,
						&isRelevant);
				if (error != PicamError_None) {
					Picam_GetEnumerationString(PicamEnumeratedType_Error,
							error,
							&errorString);
					asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
							"%s:%s Trouble getting parameter assocoated with "
							"driver param %d, picam param:%d: %s\n",
							driverName,
							functionName,
							function,
							picamParameter,
							errorString);
					Picam_DestroyString(errorString);
					return asynError;
				}
				PicamConstraintType constraintType;
				error = Picam_GetParameterConstraintType(currentCameraHandle,
						picamParameter,
						&constraintType);
				if (isRelevant &&
						(constraintType==PicamConstraintType_Collection)) {
					error = Picam_GetParameterCollectionConstraint(
							currentCameraHandle,
							picamParameter,
							PicamConstraintCategory_Capable, &constraints);
					if (error != PicamError_None){
						Picam_GetEnumerationString(PicamEnumeratedType_Error,
								error,
								&errorString);
						asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
								"%s:%s Trouble getting parameter assocoated "
								"with driver param %d, picam param:%d: %s\n",
								driverName,
								functionName,
								function,
								picamParameter,
								errorString);
						Picam_DestroyString(errorString);
					}
					if ( constraints->values_count == 0){
						strings[0] = epicsStrDup(NAString);
						values[0] = 0;
						severities[0] = 0;
						(*nIn) = 1;
						return asynSuccess;
					}
					for (int ii = 0; ii < constraints->values_count; ii++) {
						PicamEnumeratedType picamParameterET;
						Picam_GetParameterEnumeratedType(currentCameraHandle,
								picamParameter,
								&picamParameterET);
						PicamValueType valType;
						Picam_GetParameterValueType(currentCameraHandle,
								picamParameter, &valType);
						if (strings[*nIn])
							free(strings[*nIn]);
						switch (valType)
						{
						case PicamValueType_Enumeration:
							Picam_GetEnumerationString(
								picamParameterET,
								(int)constraints->values_array[ii],
								&paramConstraintString);
							asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
									"%s:%s ---%s\n",
									driverName,
									functionName,
									paramConstraintString);
							strings[*nIn] = epicsStrDup(paramConstraintString);
							values[*nIn] = (int)constraints->values_array[ii];
							Picam_DestroyString(paramConstraintString);
							severities[*nIn] = 0;
							(*nIn)++;
							break;
						case PicamValueType_FloatingPoint:
							char floatString[12];
							sprintf(floatString, "%f",
									constraints->values_array[ii]);
							strings[*nIn] = epicsStrDup(floatString);
							values[*nIn] = ii;
							severities[*nIn] = 0;
							(*nIn)++;
							break;
						case PicamValueType_Integer:
							char intString[12];
							sprintf(intString, "%d",
									(int)constraints->values_array[ii]);
							printf("---%s\n", intString);
							strings[*nIn] = epicsStrDup(intString);
							values[*nIn] = (int)constraints->values_array[ii];
							severities[*nIn] = 0;
							(*nIn)++;
							break;
						case PicamValueType_LargeInteger:
							char largeIntString[12];
							sprintf(largeIntString, "%d",
									(pi64s)constraints->values_array[ii]);
							printf("---%s\n", largeIntString);
							strings[*nIn] = epicsStrDup(largeIntString);
							values[*nIn] = (int)constraints->values_array[ii];
							severities[*nIn] = 0;
							(*nIn)++;
							break;
						case PicamValueType_Boolean:
							strings[*nIn] = epicsStrDup(
									constraints->values_array[ii] ? "No":"Yes");
							values[*nIn] = constraints->values_array[ii];
							severities[*nIn] = 0;
							(*nIn)++;
							break;
						default:
							asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
									"%s:%s Unhandled CollectionType for "
									"driverParam %d",
									driverName,
									functionName,
									function);
							return asynError;
						}
					}
				}
				else if (!isRelevant &&
						(constraintType==PicamConstraintType_Collection)){
					asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
							"Found not relevant & constraintType_Collection\n");
					strings[0] = epicsStrDup(NAString);
					values[0] = 0;
					severities[0] = 0;
					(*nIn) = 1;
				}
			}
        }
        else {
        	Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
        			picamParameter,
					&parameterName);
        	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
					"%s:%s Parameter Does Not Exist for this detector %s\n",
					driverName,
					functionName,
					parameterName);
        	Picam_DestroyString(parameterName);
			strings[0] = epicsStrDup(NAString);
			values[0] = 0;
			severities[0] = 0;
			(*nIn) = 1;
        }
    } else {
        return ADDriver::readEnum(pasynUser, strings, values, severities,
                nElements, nIn);
    }
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling enum functions, status=%d\n", driverName,
                functionName, status);
    }
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: exit\n", driverName,
            functionName);
    return asynSuccess;
}

/**
 * Read String information for fields with Enumeration type and no
 * constraint type.
 */
asynStatus ADPICam::readOctet(asynUser *pasynUser, char *value,
                                    size_t nChars, size_t *nActual,
									int *eomReason)
{
	static const char *functionName = "writeOctet";
	int status = asynSuccess;
	pibln parameterDoesExist;
	pibln parameterRelevant;
	piint intValue;
	int function = pasynUser->reason;
	PicamParameter picamParameter;
	PicamValueType valueType;
	PicamEnumeratedType enumType;
	PicamError error;
	const char *errString;
	const char *enumString;

	if ( piLookupPICamParameter(function, picamParameter) == PicamError_None) {
		Picam_DoesParameterExist(currentCameraHandle,
				picamParameter,
				&parameterDoesExist);
		if (parameterDoesExist){
			error = Picam_IsParameterRelevant(currentCameraHandle,
					picamParameter,
					&parameterRelevant);
		}
		if (error != PicamError_None){
			Picam_GetEnumerationString(PicamEnumeratedType_Error,
					error, &errString);
			asynPrint(pasynUser, ASYN_TRACE_ERROR,
					"%s:%s Trouble determining if parameter is relevant: %s\n");
			return asynError;
		}
		if (parameterRelevant){
			Picam_GetParameterValueType(currentCameraHandle,
					picamParameter,
					&valueType);
			switch (valueType) {
			case PicamValueType_Enumeration:
				Picam_GetParameterEnumeratedType(currentCameraHandle,
						picamParameter,
						&enumType);
				Picam_GetParameterIntegerValue(currentCameraHandle,
						picamParameter,
						&intValue);
				Picam_GetEnumerationString(enumType, intValue, &enumString);
				asynPrint (pasynUserSelf, ASYN_TRACE_FLOW,
						"%s:%s ----readOctet value=%s\n",
						driverName,
						functionName,
						enumString);
				lock();
				strncpy (value, enumString, nChars);
				unlock();
				value[nChars-1] = '\0';
				*nActual = strlen(value);
				Picam_DestroyString(enumString);
				break;
			}
		}
	}
	else  {
        /* If this parameter belongs to a base class call its method */
        if (function < PICAM_FIRST_PARAM) {
            status = ADDriver::readOctet(pasynUser, value, nChars, nActual,
            		eomReason);
        }
	}

	return (asynStatus)status;
}

/**
 * Overload method for asynPortDriver's report method
 */
void ADPICam::report(FILE *fp, int details) {
    static const char *functionName = "report";
    PicamError error = PicamError_None;
    pibln picamInitialized;
    const char *modelString;
    char enumString[64];
    const PicamRoisConstraint *roisConstraints;
    const char *errorString;

    fprintf(fp, "############ %s:%s ###############\n", driverName,
            functionName);
    Picam_IsLibraryInitialized(&picamInitialized);
    if (!picamInitialized) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Error: Picam is not initialized!\n", driverName,
                functionName);
        return;

    }
    char versionNumber[40];
    getStringParam(PICAM_VersionNumber, 40, versionNumber);
    fprintf(fp, "%s:%s Initialized PICam Version %s\n", driverName,
            functionName, versionNumber);
    fprintf(fp, "----------------\n");
    fprintf(fp, "%s:%s availableCamerasCount %d\n", driverName, functionName,
            availableCamerasCount);
    for (int ii = 0; ii < availableCamerasCount; ii++) {
        fprintf(fp, "Available Camera[%d]\n", ii);
        Picam_GetEnumerationString(PicamEnumeratedType_Model,
                (piint) availableCameraIDs[ii].model, &modelString);
        sprintf(enumString, "%s", modelString);
        fprintf(fp, "\n---%s\n---%d\n---%s\n---%s\n", modelString,
                availableCameraIDs[ii].computer_interface,
                availableCameraIDs[ii].sensor_name,
                availableCameraIDs[ii].serial_number);
    }
    fprintf(fp, "----------------\n");
    fprintf(fp, "%s:%s unavailableCamerasCount %d\n", driverName, functionName,
            unavailableCamerasCount);
    for (int ii = 0; ii < unavailableCamerasCount; ii++) {
        fprintf(fp, "Unavailable Camera[%d]\n", ii);
        Picam_GetEnumerationString(PicamEnumeratedType_Model,
                (piint) unavailableCameraIDs[ii].model, &modelString);
        sprintf(enumString, "%s", modelString);
        fprintf(fp, "\n---%s\n---%d\n---%s\n---%s\n", modelString,
                unavailableCameraIDs[ii].computer_interface,
                unavailableCameraIDs[ii].sensor_name,
                unavailableCameraIDs[ii].serial_number);
    }
    fprintf(fp, "----------------\n");

    if (details > 7) {
        fprintf(fp, "--------------------\n");
        fprintf(fp, " Parameters\n");
        const PicamParameter *parameterList;
        PicamConstraintType constraintType;
        piint parameterCount;
        Picam_GetParameters(currentCameraHandle, &parameterList,
                &parameterCount);
        for (int ii = 0; ii < parameterCount; ii++) {
            const char *parameterName;
            Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
                    parameterList[ii], &parameterName);

            fprintf(fp, "param[%d] %s\n", parameterList[ii], parameterName);
            Picam_GetParameterConstraintType(currentCameraHandle,
                    parameterList[ii], &constraintType);
            switch (constraintType) {
            case PicamConstraintType_Range:
                const PicamRangeConstraint *constraint;
                Picam_GetParameterRangeConstraint(currentCameraHandle,
                        parameterList[ii], PicamConstraintCategory_Capable,
                        &constraint);
                fprintf(fp, "--- Range Constraint\n");
                fprintf(fp, "------Empty Set %d\n", constraint->empty_set);
                fprintf(fp, "------Minimum %f\n", constraint->minimum);
                fprintf(fp, "------Maximum %f\n", constraint->maximum);
                fprintf(fp, "------%d excluded values\n--------",
                        constraint->excluded_values_count);
                for (int ec = 0; ec < constraint->excluded_values_count; ec++) {
                    fprintf(fp, "%f, ", constraint->excluded_values_array[ec]);
                }
                fprintf(fp, "\n");
                fprintf(fp, "------%d outlying values\n--------",
                        constraint->outlying_values_count);
                for (int ec = 0; ec < constraint->outlying_values_count; ec++) {
                    fprintf(fp, "%f, ", constraint->outlying_values_array[ec]);
                }
                fprintf(fp, "\n");
                Picam_DestroyRangeConstraints(constraint);
                break;
            case PicamConstraintType_Rois:
                error = Picam_GetParameterRoisConstraint(currentCameraHandle,
                        PicamParameter_Rois, PicamConstraintCategory_Required,
                        &roisConstraints);
                if (error != PicamError_None) {
                    Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                            &errorString);
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                            "%s:%s Error retrieving rois constraints %s\n",
                            driverName, functionName, errorString);
                    Picam_DestroyString(errorString);
                }
                fprintf(fp, "--- ROI Constraints\n"
                        "----- X min %d, X max %d\n"
                        "----- Y min %d, Y max %d\n"
                        "----- width min %d,  width max %d\n"
                        "----- height %d, height max %d\n"
                        "----- Rules 0x%x\n",
                        //			"--X bin min %d, X bin max %d\n"
                        //			"--Y bin min %d, Y bin max %d\n",
                        roisConstraints->x_constraint.minimum,
                        roisConstraints->x_constraint.maximum,
                        roisConstraints->y_constraint.minimum,
                        roisConstraints->y_constraint.maximum,
                        roisConstraints->width_constraint.minimum,
                        roisConstraints->width_constraint.maximum,
                        roisConstraints->height_constraint.minimum,
                        roisConstraints->height_constraint.maximum,
                        roisConstraints->rules);
				fprintf(fp, "-----x_binning_limits:\n");
				for (int jj = 0; jj < roisConstraints->x_binning_limits_count; jj++) {
					fprintf(fp, "------ %d\n",
							roisConstraints->x_binning_limits_array[jj]);
				}
				fprintf(fp, "-----y_binning_limits:\n");
				for (int kk = 0; kk < roisConstraints->y_binning_limits_count; kk++) {
					fprintf(fp, "------ %d\n",
							roisConstraints->y_binning_limits_array[kk]);
				}

                //			roisConstraints->x_.minimum,
                //			roisConstraints->x_constraint.minimum);
                Picam_DestroyRoisConstraints(roisConstraints);
                break;
            case PicamConstraintType_Collection:
                fprintf(fp, "----Collection Constraints\n");
                const PicamCollectionConstraint *collectionConstraint;
                Picam_GetParameterCollectionConstraint(currentCameraHandle,
                        parameterList[ii], PicamConstraintCategory_Capable,
                        &collectionConstraint);
                PicamValueType valType;
                Picam_GetParameterValueType(currentCameraHandle,
                        parameterList[ii], &valType);
                switch (valType) {
                case PicamValueType_Enumeration:
                    for (int cc = 0; cc < collectionConstraint->values_count;
                            cc++) {
                        PicamEnumeratedType enumeratedType;
                        const pichar *enumerationString;
                        Picam_GetParameterEnumeratedType(currentCameraHandle,
                                parameterList[ii], &enumeratedType);
                        Picam_GetEnumerationString(enumeratedType,
                                (int) collectionConstraint->values_array[cc],
                                &enumerationString);
                        fprintf(fp, "------ %s\n", enumerationString);
                        Picam_DestroyString(enumerationString);
                    }
                    break;
                case PicamValueType_FloatingPoint:
                    for (int cc = 0; cc < collectionConstraint->values_count;
                            cc++) {
                        fprintf(fp, "------ %f\n",
                                collectionConstraint->values_array[cc]);
                    }
                    break;
                case PicamValueType_Integer:
                    for (int cc = 0; cc < collectionConstraint->values_count;
                            cc++) {
                        fprintf(fp, "------ %d\n",
                                (piint) collectionConstraint->values_array[cc]);
                    }
                    break;
                case PicamValueType_LargeInteger:
                    for (int cc = 0; cc < collectionConstraint->values_count;
                            cc++) {
                        fprintf(fp, "------ %d\n",
                                (pi64s) collectionConstraint->values_array[cc]);
                    }
                    break;
                case PicamValueType_Boolean:
                    for (int cc = 0; cc < collectionConstraint->values_count;
                            cc++) {
                        fprintf(fp, "------ %s\n",
                                collectionConstraint->values_array[cc] ?
                                        "true" : "false");
                    }
                    break;
                default:
                    fprintf(fp,
                            "---Unhandled valueType for collection constraint");
                }
                Picam_DestroyCollectionConstraints(collectionConstraint);
                break;
            default:
                break;
            }
        }
        ADDriver::report(fp, details);
    }

    if (details > 20) {
        piint demoModelCount;
        const PicamModel * demoModels;

        error = Picam_GetAvailableDemoCameraModels(&demoModels,
                &demoModelCount);
        for (int ii = 0; ii < demoModelCount; ii++) {
            const char* modelString;
            error = Picam_GetEnumerationString(PicamEnumeratedType_Model,
                    demoModels[ii], &modelString);
            fprintf(fp, "demoModel[%d]: %s\n", ii, modelString);
            Picam_DestroyString(modelString);
        }
        Picam_DestroyModels(demoModels);

    }
}

/**
 * Overload asynPortDriver's writeFloat64 to handle driver specific
 * parameters.
 */
asynStatus ADPICam::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    static const char *functionName = "writeFloat64";
    int status = asynSuccess;
    PicamError error = PicamError_None;
    const char *errorString;
    int function = pasynUser->reason;
    PicamParameter picamParameter;

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: entry\n", driverName,
            functionName);

    if (piLookupPICamParameter(function, picamParameter) ==
    		PicamError_None){
    	pibln isRelevant;
    	error = Picam_IsParameterRelevant(currentCameraHandle,
    			picamParameter,
				&isRelevant);
    	if (error == PicamError_ParameterDoesNotExist){
    		isRelevant = false;
    	}
    	else if (error != PicamError_None) {
    		Picam_GetEnumerationString(PicamEnumeratedType_Error,
    				error,
					&errorString);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s Trouble getting parameter associated with driver"
                    " param %d, picam param:%d: %s\n",
                    driverName,
                    functionName,
                    function,
                    picamParameter,
                    errorString);
            Picam_DestroyString(errorString);
            return asynError;
    	}
    	if (isRelevant && currentCameraHandle != NULL) {
    		PicamConstraintType constraintType;
    		error = Picam_GetParameterConstraintType(currentCameraHandle,
    				picamParameter,
					&constraintType);
            if (error != PicamError_None){
                Picam_GetEnumerationString(PicamEnumeratedType_Error,
                        error,
                        &errorString);
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s Trouble getting parameter assocoated with driver"
                        " param %d, picam param:%d: %s\n",
                        driverName,
                        functionName,
                        function,
                        picamParameter,
                        errorString);
                Picam_DestroyString(errorString);
            }
            if (constraintType==PicamConstraintType_Range) {
                status |= piWriteFloat64RangeType(pasynUser,
                        value,
                        function,
                        picamParameter);
            }
            else if (constraintType == PicamConstraintType_Collection) {
				error = Picam_SetParameterFloatingPointValue(currentCameraHandle,
						picamParameter, (piflt) value);
				if (error != PicamError_None) {
					Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
							&errorString);

					asynPrint(pasynUser, ASYN_TRACE_ERROR,
							"%s:%s error writing Float64 value to %f\n"
							"Reason %s\n",
							driverName,
							functionName,
							value,
							errorString);
					Picam_DestroyString(errorString);
					return asynError;
				}

            }
            else if (constraintType==PicamConstraintType_None) {
                status |= Picam_SetParameterFloatingPointValue(
                		currentCameraHandle,
						picamParameter,
						value);
            }
    	}
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < PICAM_FIRST_PARAM) {
            status = ADDriver::writeFloat64(pasynUser, value);
        }

    }
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s: error, status=%d function=%d, value=%f\n", driverName,
                functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, value=%f\n", driverName, functionName,
                function, value);
    return (asynStatus) status;
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: exit\n", driverName,
            functionName);

}

/**
 * Override asynPortDriver's writeInt32 method.
 */
asynStatus ADPICam::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    static const char *functionName = "writeInt32";
    int status = asynSuccess;
    PicamError error = PicamError_None;
    const char* errorString;
    int sizeX, sizeY, binX, binY, minX, minY;
    int function = pasynUser->reason;
    PicamParameter picamParameter;

    if (function == PICAM_AvailableCameras) {
        piSetSelectedCamera(pasynUser, (int) value);
        piSetParameterValuesFromSelectedCamera();
    } else if (function == PICAM_UnavailableCameras) {
        piSetSelectedUnavailableCamera(pasynUser, (int) value);
    } else if (piLookupPICamParameter(function, picamParameter) ==
            PicamError_None) {
        pibln isRelevant;
        error = Picam_IsParameterRelevant(currentCameraHandle,
                picamParameter,
                &isRelevant);
        if (error == PicamError_ParameterDoesNotExist) {
            isRelevant = false;
        }
        else if (error != PicamError_None) {
                Picam_GetEnumerationString(PicamEnumeratedType_Error,
                    error,
                    &errorString);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s Trouble getting parameter associated with driver"
                    " param %d, picam param:%d: %s\n",
                    driverName,
                    functionName,
                    function,
                    picamParameter,
                    errorString);
            Picam_DestroyString(errorString);
            return asynError;
        }
        if (isRelevant && currentCameraHandle != NULL){
            PicamConstraintType constraintType;
            error = Picam_GetParameterConstraintType(currentCameraHandle,
                    picamParameter,
                    &constraintType);
            if (error != PicamError_None){
                Picam_GetEnumerationString(PicamEnumeratedType_Error,
                        error,
                        &errorString);
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s Trouble getting parameter assocoated with driver"
                        " param %d, picam param:%d: %s\n",
                        driverName,
                        functionName,
                        function,
                        picamParameter,
                        errorString);
                Picam_DestroyString(errorString);
            }

            if (constraintType==PicamConstraintType_Range) {
                status |= piWriteInt32RangeType(pasynUser,
                        value,
                        function,
                        picamParameter);

            }
            else if (constraintType==PicamConstraintType_Collection) {
                status |= piWriteInt32CollectionType(pasynUser,
                        value,
                        function,
                        picamParameter);
            }

        }
    }
    // AD parameters for ROI size & Bin size need to be mapped into
    // PICAM's PicamRois object.
    else if (function == ADSizeX) {
        int xstatus = asynSuccess;
        sizeX = value;
        xstatus |= getIntegerParam(ADSizeY, &sizeY);
        xstatus |= getIntegerParam(ADBinX, &binX);
        xstatus |= getIntegerParam(ADBinY, &binY);
        xstatus |= getIntegerParam(ADMinX, &minX);
        xstatus |= getIntegerParam(ADMinY, &minY);
        xstatus |= piSetRois(minX, minY, sizeX, sizeY, binX, binY);
    } else if (function == ADSizeY) {
        int xstatus = asynSuccess;
        xstatus |= getIntegerParam(ADSizeX, &sizeX);
        sizeY = value;
        xstatus |= getIntegerParam(ADBinX, &binX);
        xstatus |= getIntegerParam(ADBinY, &binY);
        xstatus |= getIntegerParam(ADMinX, &minX);
        xstatus |= getIntegerParam(ADMinY, &minY);
        xstatus |= piSetRois(minX, minY, sizeX, sizeY, binX, binY);
    } else if (function == ADBinX) {
        int xstatus = asynSuccess;
        xstatus |= getIntegerParam(ADSizeX, &sizeX);
        xstatus |= getIntegerParam(ADSizeY, &sizeY);
        binX = value;
        xstatus |= getIntegerParam(ADBinY, &binY);
        xstatus |= getIntegerParam(ADMinX, &minX);
        xstatus |= getIntegerParam(ADMinY, &minY);
        xstatus |= piSetRois(minX, minY, sizeX, sizeY, binX, binY);
    } else if (function == ADBinY) {
        int xstatus = asynSuccess;
        xstatus |= getIntegerParam(ADSizeX, &sizeX);
        xstatus |= getIntegerParam(ADSizeY, &sizeY);
        xstatus |= getIntegerParam(ADBinX, &binX);
        binY = value;
        xstatus |= getIntegerParam(ADMinX, &minX);
        xstatus |= getIntegerParam(ADMinY, &minY);
        xstatus |= piSetRois(minX, minY, sizeX, sizeY, binX, binY);
    } else if (function == ADMinX) {
        int xstatus = asynSuccess;
        xstatus |= getIntegerParam(ADSizeX, &sizeX);
        xstatus |= getIntegerParam(ADSizeY, &sizeY);
        xstatus |= getIntegerParam(ADBinX, &binX);
        xstatus |= getIntegerParam(ADBinY, &binY);
        minX = value;
        xstatus |= getIntegerParam(ADMinY, &minY);
        xstatus |= piSetRois(minX, minY, sizeX, sizeY, binX, binY);
    } else if (function == ADMinY) {
        int xstatus = asynSuccess;
        xstatus |= getIntegerParam(ADSizeX, &sizeX);
        xstatus |= getIntegerParam(ADSizeY, &sizeY);
        xstatus |= getIntegerParam(ADBinX, &binX);
        xstatus |= getIntegerParam(ADBinY, &binY);
        xstatus |= getIntegerParam(ADMinX, &minX);
        minY = value;
        xstatus |= piSetRois(minX, minY, sizeX, sizeY, binX, binY);
    } else if (function == ADAcquire) {
        int adStatus;
        getIntegerParam(ADStatus, &adStatus);
        printf ("Caught acquire value %d, status = %d\n",
                value, adStatus);
        if (value && (adStatus == ADStatusIdle)){
            piAcquireStart();
        }
        else if (!value && (adStatus != ADStatusIdle)){
            piAcquireStop();
        }

    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < PICAM_FIRST_PARAM) {
            status = ADDriver::writeInt32(pasynUser, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                "%s:%s: error, status=%d function=%d, value=%d\n", driverName,
                functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, value=%d\n", driverName, functionName,
                function, value);

    return (asynStatus) status;

}

/**
 * Internal method called when the Acquire button is pressed.
 */
asynStatus ADPICam::piAcquireStart(){
    const char *functionName = "piAcquireStart";
    int status = asynSuccess;
    PicamError error = PicamError_None;
    int imageMode;
    int presetImages;
    int numX;
    int numY;

    // Reset the number of Images Collected
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s Enter\n");
    lock();
    setIntegerParam(ADStatus, ADStatusInitializing);
    // reset Image counter
    setIntegerParam(ADNumImagesCounter, 0);
    callParamCallbacks();
    unlock();
    getIntegerParam(ADImageMode, &imageMode);

    /* Get Image size for use by acquisition handling*/
    getIntegerParam(ADSizeX, &numX);
    getIntegerParam(ADSizeY, &numY);
    imageDims[0] = numX;
    imageDims[1] = numY;

    /* get data type for acquistion processing */
    piint pixelFormat;
    Picam_GetParameterIntegerDefaultValue(currentCameraHandle,
            PicamParameter_PixelFormat,
            &pixelFormat);
    switch(pixelFormat){
    case PicamPixelFormat_Monochrome16Bit:
        imageDataType = NDUInt16;
        break;
    default:
        imageDataType = NDUInt16;
        const char *pixelFormatString;
        Picam_GetEnumerationString(PicamEnumeratedType_PixelFormat,
                pixelFormat, &pixelFormatString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Unknown data type setting to NDUInt16: %s\n",
                driverName, functionName, pixelFormatString);
        Picam_DestroyString(pixelFormatString);
    }


    switch(imageMode) {
    case ADImageSingle:
        presetImages = 1;
        break;
    case ADImageMultiple:
        getIntegerParam(ADNumImages, &presetImages);
        break;
    case ADImageContinuous:
        presetImages = 0;
        break;

    }

    pi64s largePreset;
    largePreset = presetImages;
    Picam_SetParameterLargeIntegerValue(currentCameraHandle,
            PicamParameter_ReadoutCount,
            largePreset);
    int readoutStride;
    double onlineReadoutRate;
    int timeStampsUsed;

    Picam_GetParameterIntegerValue(currentCameraHandle,
    		PicamParameter_ReadoutStride,
			&readoutStride);
    Picam_GetParameterFloatingPointValue(currentCameraHandle,
    		PicamParameter_OnlineReadoutRateCalculation,
			&onlineReadoutRate);
    Picam_GetParameterIntegerValue(currentCameraHandle,
    		PicamParameter_TimeStamps,
			&timeStampsUsed);
    pi64s readouts =
    		static_cast<pi64s>(std::ceil(std::max(6.*onlineReadoutRate, 6.)));
    buffer_.resize(readouts * (readoutStride+3*8));
    PicamAcquisitionBuffer piBuffer;
    piBuffer.memory = &buffer_[0];
    piBuffer.memory_size = buffer_.size();

    error = PicamAdvanced_SetAcquisitionBuffer(currentDeviceHandle, &piBuffer);
    if (error != PicamError_None) {
        const char *errorString;
        Picam_GetEnumerationString(PicamEnumeratedType_Error,
                error,
                &errorString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Error Setting accquisition buffer with size %d: %s\n",
                driverName,
                functionName,
                errorString,
                buffer_.size());
        Picam_DestroyString(errorString);

    }

    lock();
    setIntegerParam(ADStatus, ADStatusAcquire);
    callParamCallbacks();
    unlock();
    const PicamParameter *failedParameterArray;
    piint failedParameterCount;
    Picam_CommitParameters(currentCameraHandle,
            &failedParameterArray,
            &failedParameterCount);
    error = Picam_StartAcquisition(currentCameraHandle);
    if (error != PicamError_None){
        const char *errorString;
        Picam_GetEnumerationString(PicamEnumeratedType_Error,
                error,
                &errorString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Error with Picam_StartAcquisition: %s\n",
                driverName,
                functionName,
                errorString);
        Picam_DestroyString(errorString);
    }

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s Enter\n");

    return (asynStatus)status;
}

/**
 * Internal method called when stop acquire is pressed.
 */
asynStatus ADPICam::piAcquireStop(){
    const char *functionName = "piAcquireStop";
    int status = asynSuccess;

    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "Stop acquisition\n");

    pibln isRunning = false;
    Picam_IsAcquisitionRunning(currentCameraHandle, &isRunning);
    if (isRunning) {
        Picam_StopAcquisition(currentDeviceHandle);
    }
    lock();
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(ADStatus, ADStatusIdle);
    callParamCallbacks();
    unlock();

    return (asynStatus)status;

}

/**
 * Callback method for acquisition Upadated event.  This will call
 * piHandleAcquisitionUpdated ASAP.
 */
PicamError PIL_CALL ADPICam::piAcquistionUpdated(
        PicamHandle device,
        const PicamAvailableData *available,
        const PicamAcquisitionStatus *acqStatus)
{
    int status = asynSuccess;
    PicamError error = PicamError_None;
    const char *functionName = "piParameterIntegerValueChanged";
    //ADPICam_Instance->piSetAcquisitionData(device, available, acqStatus );
    status = ADPICam_Instance->piHandleAcquisitionUpdated(device,
            available, acqStatus);

    return error;
}

/**
 * Local Method used to add a Demo camera to the list of available
 * cameras.  This method is called by wrapper method PICamAddDemoCamera
 * which can be called from the iocsh.
 */
asynStatus ADPICam::piAddDemoCamera(const char *demoCameraName) {
    const char * functionName = "piAddDemoCamera";
    int status = asynSuccess;
    PicamError error = PicamError_None;
    const PicamModel *demoModels;
    piint demoModelCount;
    bool modelFoundInList = false;
    PicamCameraID demoID;
    const char *errorString;

    pibln libInitialized = true;
    Picam_IsLibraryInitialized(&libInitialized);

    if (libInitialized) {
        error = Picam_GetAvailableDemoCameraModels(&demoModels,
                &demoModelCount);
        for (int ii = 0; ii < demoModelCount; ii++) {
            const char* modelString;
            error = Picam_GetEnumerationString(PicamEnumeratedType_Model,
                    demoModels[ii], &modelString);
            if (strcmp(demoCameraName, modelString) == 0) {
                modelFoundInList = true;
                error = Picam_ConnectDemoCamera(demoModels[ii], "ADDemo",
                        &demoID);
                if (error == PicamError_None) {
                    printf("%s:%s Adding camera demoCamera[%d] %s\n",
                            driverName, functionName, demoModels[ii],
                            demoCameraName);
                    //ADPICam_Instance->piUpdateAvailableCamerasList();
                } else {
                    Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                            &errorString);
                    printf("%s:%s Error Adding camera demoCamera[%d] %s, %s\n",
                            driverName, functionName, ii, demoCameraName,
                            errorString);
                }
                ii = demoModelCount;
            }
            Picam_DestroyString(modelString);
        }
        Picam_DestroyModels(demoModels);
    } else {
        status = asynError;
    }

    return (asynStatus) status;
}

/**
 * Callback method for camera discovery.  This method calls the
 * piHandleCameraDiscovery method of the camera instance.
 */
PicamError PIL_CALL ADPICam::piCameraDiscovered(const PicamCameraID *id,
        PicamHandle device, PicamDiscoveryAction action) {
    int status;
    status = ADPICam_Instance->piHandleCameraDiscovery(id, device, action);

    return PicamError_None;
}

/**
 * Set all PICAM parameter relevance parameters to false
 */
asynStatus ADPICam::piClearParameterExists() {
    int iParam;
    int status = asynSuccess;

    for (iParam = PICAM_ExposureTimeExists;
            iParam <= PICAM_SensorTemperatureStatusExists; iParam++) {
        status |= setIntegerParam(iParam, 0);
    }
    return (asynStatus) status;
}

/**
 Set all PICAM parameter relevance parameters to false
 */
asynStatus ADPICam::piClearParameterRelevance() {
    int iParam;
    int status = asynSuccess;

    for (iParam = PICAM_ExposureTimeRelevant;
            iParam <= PICAM_SensorTemperatureStatusRelevant; iParam++) {
        status |= setIntegerParam(iParam, 0);
    }
    return (asynStatus) status;
}

/**
 *
 */
asynStatus ADPICam::piLoadAvailableCameraIDs() {
    const char *functionName = "piLoadAvailableCameraIDs";
    int status = asynSuccess;
    PicamError error = PicamError_None;
    const char *errorString;

    if (availableCamerasCount != 0) {
        error = Picam_DestroyCameraIDs(availableCameraIDs);
        availableCamerasCount = 0;
        if (error != PicamError_None) {
            Picam_GetEnumerationString(PicamEnumeratedType_Error, (piint) error,
                    &errorString);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Picam_DestroyCameraIDs error %s", driverName,
                    functionName, errorString);
            Picam_DestroyString(errorString);
            return asynError;
        }
    }
    error = Picam_GetAvailableCameraIDs(&availableCameraIDs,
            &availableCamerasCount);
    if (error != PicamError_None) {
        Picam_GetEnumerationString(PicamEnumeratedType_Error, (piint) error,
                &errorString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s%s: Picam_GetAvailableCameraIDs error %s", driverName,
                functionName, errorString);
        Picam_DestroyString(errorString);
        return asynError;
    }
    return (asynStatus) status;
}

asynStatus ADPICam::piLoadUnavailableCameraIDs() {
    PicamError error;
    const char *errorString;
    const char *functionName = "piLoadUnavailableCameraIDs";
    int status = asynSuccess;

    if (unavailableCamerasCount != 0) {
        error = Picam_DestroyCameraIDs(unavailableCameraIDs);
        unavailableCamerasCount = 0;
        if (error != PicamError_None) {
            Picam_GetEnumerationString(PicamEnumeratedType_Error, (piint) error,
                    &errorString);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Picam_DestroyCameraIDs error %s", driverName,
                    functionName, errorString);
            Picam_DestroyString(errorString);
            return asynError;
        }
    }
    error = Picam_GetUnavailableCameraIDs(&unavailableCameraIDs,
            &unavailableCamerasCount);
    if (error != PicamError_None) {
        Picam_GetEnumerationString(PicamEnumeratedType_Error, (piint) error,
                &errorString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s%s: Picam_GetUnavailableCameraIDs error %s", driverName,
                functionName, errorString);
        Picam_DestroyString(errorString);
        return asynError;
    }
    return (asynStatus) status;
}

/**
 * Given a PICAM library Parameter, return the associated areaDetector Driver
 * Parameter.
 */
int ADPICam::piLookupDriverParameter(PicamParameter parameter) {
    const char *functionName = "piLookupDriverParameter";
    int driverParameter = -1;
    const char *paramString;

    switch (parameter) {
    case PicamParameter_Accumulations:
        driverParameter = ADNumExposures;
        break;
    case PicamParameter_ActiveBottomMargin:
    	driverParameter = PICAM_ActiveBottomMargin;
        break;
    case PicamParameter_ActiveHeight:
    	driverParameter = PICAM_ActiveHeight;
        break;
    case PicamParameter_ActiveLeftMargin:
    	driverParameter = PICAM_ActiveLeftMargin;
        break;
    case PicamParameter_ActiveRightMargin:
    	driverParameter = PICAM_ActiveRightMargin;
        break;
    case PicamParameter_ActiveTopMargin:
    	driverParameter = PICAM_ActiveTopMargin;
        break;
    case PicamParameter_ActiveWidth:
    	driverParameter = PICAM_ActiveWidth;
        break;
    case PicamParameter_AdcAnalogGain:
        driverParameter = PICAM_AdcAnalogGain;
        break;
    case PicamParameter_AdcBitDepth:
        driverParameter = PICAM_AdcBitDepth;
        break;
    case PicamParameter_AdcEMGain:
    	driverParameter = PICAM_AdcEMGain;
        break;
    case PicamParameter_AdcQuality:
        driverParameter = PICAM_AdcQuality;
        break;
    case PicamParameter_AdcSpeed:
        driverParameter = PICAM_AdcSpeed;
        break;
    case PicamParameter_AuxOutput:
    	driverParameter = PICAM_AuxOutput;
        break;
    case PicamParameter_BracketGating:
        break;
    case PicamParameter_CcdCharacteristics:
    	driverParameter = PICAM_CcdCharacteristics;
        break;
    case PicamParameter_CleanBeforeExposure:
    	driverParameter = PICAM_CleanBeforeExposure;
        break;
    case PicamParameter_CleanCycleCount:
        driverParameter = PICAM_CleanCycleCount;
    	break;
    case PicamParameter_CleanCycleHeight:
        driverParameter = PICAM_CleanCycleCount;
    	break;
    case PicamParameter_CleanSectionFinalHeight:
        driverParameter = PICAM_CleanSectionFinalHeight;
    	break;
    case PicamParameter_CleanSectionFinalHeightCount:
        driverParameter = PICAM_CleanSectionFinalHeightCount;
    	break;
    case PicamParameter_CleanSerialRegister:
    	driverParameter = PICAM_CleanSerialRegister;
        break;
    case PicamParameter_CleanUntilTrigger:
    	driverParameter = PICAM_CleanUntilTrigger;
        break;
    case PicamParameter_CorrectPixelBias:
        break;
    case PicamParameter_CustomModulationSequence:
        break;
    case PicamParameter_DifEndingGate:
        break;
    case PicamParameter_DifStartingGate:
        break;
    case PicamParameter_DisableCoolingFan:
    	driverParameter = PICAM_DisableCoolingFan;
    	break;
    case PicamParameter_DisableDataFormatting:
        driverParameter = PICAM_DisableDataFormatting;
        break;
    case PicamParameter_EMIccdGain:
        break;
    case PicamParameter_EMIccdGainControlMode:
        break;
    case PicamParameter_EnableIntensifier:
        break;
    case PicamParameter_EnableModulation:
        break;
    case PicamParameter_EnableModulationOutputSignal:
    	driverParameter = PICAM_EnableModulationOutputSignal;
        break;
    case PicamParameter_ModulationOutputSignalFrequency:
    	driverParameter = PICAM_ModulationOutputSignalFrequency;
        break;
    case PicamParameter_ModulationOutputSignalAmplitude:
    	driverParameter = PICAM_ModulationOutputSignalAmplitude;
        break;
    case PicamParameter_EnableNondestructiveReadout:
    	driverParameter = PICAM_EnableNondestructiveReadout;
        break;
    case PicamParameter_EnableSensorWindowHeater:
    	driverParameter = PICAM_EnableSensorWindowHeater;
        break;
    case PicamParameter_EnableSyncMaster:
    	driverParameter = PICAM_EnableSyncMaster;
    	break;
    case PicamParameter_ExactReadoutCountMaximum:
        driverParameter = PICAM_ExactReadoutCountMax;
        break;
    case PicamParameter_ExposureTime:
        driverParameter = ADAcquireTime;
        break;
    case PicamParameter_FrameRateCalculation:
        driverParameter = PICAM_FrameRateCalc;
        break;
    case PicamParameter_FrameSize:
        driverParameter = NDArraySize;
        break;
    case PicamParameter_FramesPerReadout:
        driverParameter = PICAM_FramesPerReadout;
        break;
    case PicamParameter_FrameStride:
        driverParameter = PICAM_FrameStride;
        break;
    case PicamParameter_FrameTrackingBitDepth:
        driverParameter = PICAM_FrameTrackingBitDepth;
        break;
    case PicamParameter_GateTracking:
        driverParameter = PICAM_GateTracking;
        break;
    case PicamParameter_GateTrackingBitDepth:
        driverParameter = PICAM_GateTrackingBitDepth;
        break;
    case PicamParameter_GatingMode:
        break;
    case PicamParameter_GatingSpeed:
        break;
    case PicamParameter_IntensifierDiameter:
        break;
    case PicamParameter_IntensifierGain:
        break;
    case PicamParameter_IntensifierOptions:
        break;
    case PicamParameter_IntensifierStatus:
        break;
    case PicamParameter_InvertOutputSignal:
        break;
    case PicamParameter_KineticsWindowHeight:
    	driverParameter = PICAM_KineticsWindowHeight;
        break;
    case PicamParameter_MaskedBottomMargin:
    	driverParameter = PICAM_MaskedBottomMargin;
        break;
    case PicamParameter_MaskedHeight:
    	driverParameter = PICAM_MaskedHeight;
        break;
    case PicamParameter_MaskedTopMargin:
        driverParameter = PICAM_MaskedTopMargin;
    	break;
    case PicamParameter_ModulationDuration:
    	break;
    case PicamParameter_ModulationFrequency:
        break;
    case PicamParameter_ModulationTracking:
        driverParameter = PICAM_ModulationTracking;
        break;
    case PicamParameter_ModulationTrackingBitDepth:
        driverParameter = PICAM_ModulationTrackingBitDepth;
        break;
    case PicamParameter_NondestructiveReadoutPeriod:
    	driverParameter = PICAM_NondestructiveReadoutPeriod;
    	break;
    case PicamParameter_NormalizeOrientation:
        driverParameter = PICAM_NormalizeOrientation;
        break;
    case PicamParameter_OnlineReadoutRateCalculation:
        driverParameter = PICAM_OnlineReadoutRateCalc;
        break;
    case PicamParameter_Orientation:
        driverParameter = PICAM_Orientation;
        break;
    case PicamParameter_OutputSignal:
        driverParameter = PICAM_OutputSignal;
    	break;
    case PicamParameter_PhosphorDecayDelay:
        break;
    case PicamParameter_PhosphorDecayDelayResolution:
    	driverParameter = PICAM_PhosphorDecayDelayResolution;
        break;
    case PicamParameter_PhosphorType:
    	driverParameter = PICAM_PhosphorType;
        break;
    case PicamParameter_PhotocathodeSensitivity:
        break;
    case PicamParameter_PhotonDetectionMode:
        driverParameter = PICAM_PhotonDetectionMode;
        break;
    case PicamParameter_PhotonDetectionThreshold:
        driverParameter = PICAM_PhotonDetectionThreshold;
        break;
    case PicamParameter_PixelBitDepth:
        driverParameter = PICAM_PixelBitDepth;
        break;
    case PicamParameter_PixelFormat:
        driverParameter = PICAM_PixelFormat;
        break;
    case PicamParameter_PixelGapHeight:
    	driverParameter = PICAM_PixelGapHeight;
        break;
    case PicamParameter_PixelGapWidth:
    	driverParameter = PICAM_PixelGapWidth;
        break;
    case PicamParameter_PixelHeight:
    	driverParameter = PICAM_PixelHeight;
        break;
    case PicamParameter_PixelWidth:
        driverParameter = PICAM_PixelWidth;
    	break;
    case PicamParameter_ReadoutControlMode:
        driverParameter = PICAM_ReadoutControlMode;
        break;
    case PicamParameter_ReadoutCount:
        driverParameter = PICAM_ReadoutCount;
        break;
    case PicamParameter_ReadoutOrientation:
    	driverParameter = PICAM_ReadoutOrientation;
        break;
    case PicamParameter_ReadoutPortCount:
        driverParameter = PICAM_ReadoutPortCount;
    	break;
    case PicamParameter_ReadoutRateCalculation:
        driverParameter = PICAM_ReadoutRateCalc;
        break;
    case PicamParameter_ReadoutStride:
        driverParameter = PICAM_ReadoutStride;
        break;
    case PicamParameter_ReadoutTimeCalculation:
        driverParameter = PICAM_ReadoutTimeCalc;
        break;
    case PicamParameter_RepetitiveGate:
        break;
    case PicamParameter_RepetitiveModulationPhase:
        break;
    case PicamParameter_Rois:
        break;
    case PicamParameter_SecondaryActiveHeight:
        driverParameter = PICAM_SecondaryActiveHeight;
    	break;
    case PicamParameter_SecondaryMaskedHeight:
    	driverParameter = PICAM_SecondaryMaskedHeight;
        break;
    case PicamParameter_SensorActiveBottomMargin:
        driverParameter = PICAM_SensorActiveBottomMargin;
    	break;
    case PicamParameter_SensorActiveHeight:
        driverParameter = ADMaxSizeY;
        break;
    case PicamParameter_SensorActiveLeftMargin:
    	driverParameter = PICAM_SensorActiveLeftMargin;
        break;
    case PicamParameter_SensorActiveRightMargin:
    	driverParameter = PICAM_SensorActiveRightMargin;
        break;
    case PicamParameter_SensorActiveTopMargin:
    	driverParameter = PICAM_SensorActiveTopMargin;
        break;
    case PicamParameter_SensorActiveWidth:
        driverParameter = ADMaxSizeX;
        break;
    case PicamParameter_SensorMaskedBottomMargin:
    	driverParameter = PICAM_SensorMaskedBottomMargin;
        break;
    case PicamParameter_SensorMaskedHeight:
    	driverParameter = PICAM_SensorMaskedHeight;
        break;
    case PicamParameter_SensorMaskedTopMargin:
    	driverParameter = PICAM_SensorMaskedTopMargin;
        break;
    case PicamParameter_SensorSecondaryActiveHeight:
        driverParameter = PICAM_SensorSecondaryActiveHeight;
    	break;
    case PicamParameter_SensorSecondaryMaskedHeight:
        driverParameter = PICAM_SensorSecondaryMaskedHeight;
        break;
    case PicamParameter_SensorTemperatureReading:
        driverParameter = ADTemperatureActual;
        break;
    case PicamParameter_SensorTemperatureSetPoint:
        driverParameter = ADTemperature;
        break;
    case PicamParameter_SensorTemperatureStatus:
        driverParameter = PICAM_SensorTemperatureStatus;
    	break;
    case PicamParameter_SensorType:
    	driverParameter = PICAM_SensorType;
        break;
    case PicamParameter_SequentialEndingGate:
        break;
    case PicamParameter_SequentialEndingModulationPhase:
        break;
    case PicamParameter_SequentialGateStepCount:
        break;
    case PicamParameter_SequentialGateStepIterations:
        break;
    case PicamParameter_SequentialStartingGate:
        break;
    case PicamParameter_SequentialStartingModulationPhase:
        break;
    case PicamParameter_ShutterClosingDelay:
        driverParameter = ADShutterCloseDelay;
        break;
    case PicamParameter_ShutterDelayResolution:
    	driverParameter = PICAM_ShutterDelayResolution;
        break;
    case PicamParameter_ShutterOpeningDelay:
        driverParameter = ADShutterOpenDelay;
        break;
    case PicamParameter_ShutterTimingMode:
    	driverParameter = PICAM_ShutterTimingMode;
        break;
    case PicamParameter_SyncMaster2Delay:
        break;
    case PicamParameter_TimeStampBitDepth:
        driverParameter = PICAM_TimeStampBitDepth;
        break;
    case PicamParameter_TimeStampResolution:
        driverParameter = PICAM_TimeStampResolution;
        break;
    case PicamParameter_TimeStamps:
        driverParameter = PICAM_TimeStamps;
        break;
    case PicamParameter_TrackFrames:
        driverParameter = PICAM_TrackFrames;
        break;
    case PicamParameter_TriggerCoupling:
    	driverParameter = PICAM_TriggerCoupling;
        break;
    case PicamParameter_TriggerDetermination:
    	driverParameter = PICAM_TriggerDetermination;
        break;
    case PicamParameter_TriggerFrequency:
    	driverParameter = PICAM_TriggerFrequency;
        break;
    case PicamParameter_TriggerResponse:
        driverParameter = ADTriggerMode;
        break;
    case PicamParameter_TriggerSource:
    	driverParameter = PICAM_TriggerSource;
        break;
    case PicamParameter_TriggerTermination:
        driverParameter = PICAM_TriggerTermination;
    	break;
    case PicamParameter_TriggerThreshold:
    	driverParameter = PICAM_TriggerThreshold;
        break;
    case PicamParameter_VerticalShiftRate:
    	driverParameter = PICAM_VerticalShiftRate;
        break;
    default:
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
                &paramString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "---- Can't find parameter %s\n", paramString);
        Picam_DestroyString(paramString);
        break;
    }

    return driverParameter;

}

/**
 * Provide a translation between parameters for this camera (& those
 * inherited by ADDriver as necessary) and Picam's parameters.
 */
PicamError ADPICam::piLookupPICamParameter(int driverParameter,
        PicamParameter &parameter){
    const char *functionName = "piLookupPICamParameter";

    if (driverParameter ==ADNumExposures) {
        parameter = PicamParameter_Accumulations;
    }
    else if (driverParameter == PICAM_ActiveBottomMargin){
        parameter = PicamParameter_ActiveBottomMargin;
    }
    else if (driverParameter == PICAM_ActiveHeight){
        parameter = PicamParameter_ActiveHeight;
    }
    else if (driverParameter == PICAM_ActiveLeftMargin){
        parameter = PicamParameter_ActiveLeftMargin;
    }
    else if (driverParameter == PICAM_ActiveRightMargin){
        parameter = PicamParameter_ActiveRightMargin;
    }
    else if (driverParameter == PICAM_ActiveTopMargin){
        parameter = PicamParameter_ActiveTopMargin;
    }
    else if (driverParameter == PICAM_ActiveWidth){
        parameter = PicamParameter_ActiveWidth;
    }
    else if (driverParameter == PICAM_AdcAnalogGain){
        parameter = PicamParameter_AdcAnalogGain;
    }
    else if (driverParameter == PICAM_AdcBitDepth) {
        parameter = PicamParameter_AdcBitDepth;
    }
    else if (driverParameter == PICAM_AdcQuality) {
        parameter = PicamParameter_AdcQuality;
    }
    else if (driverParameter == PICAM_AdcSpeed) {
        parameter = PicamParameter_AdcSpeed;
    }
    else if (driverParameter == PICAM_AuxOutput) {
        parameter = PicamParameter_AuxOutput;
    }
    else if (driverParameter == PICAM_CleanBeforeExposure) {
        parameter = PicamParameter_CleanBeforeExposure;
    }
    else if (driverParameter == PICAM_CleanCycleCount) {
        parameter = PicamParameter_CleanCycleCount;
    }
    else if (driverParameter == PICAM_CleanCycleHeight) {
        parameter = PicamParameter_CleanCycleHeight;
    }
    else if (driverParameter == PICAM_CleanSectionFinalHeight) {
        parameter = PicamParameter_CleanSectionFinalHeight;
    }
    else if (driverParameter == PICAM_CleanSectionFinalHeightCount) {
        parameter = PicamParameter_CleanSectionFinalHeightCount;
    }
    else if (driverParameter == PICAM_CleanSerialRegister) {
        parameter = PicamParameter_CleanSerialRegister;
    }
    else if (driverParameter == PICAM_CleanUntilTrigger) {
        parameter = PicamParameter_CleanUntilTrigger;
    }
    else if (driverParameter == PICAM_CcdCharacteristics){
        parameter = PicamParameter_CcdCharacteristics;
    }
    else if (driverParameter == PICAM_DisableCoolingFan) {
        parameter = PicamParameter_DisableCoolingFan;
    }
    else if (driverParameter == PICAM_DisableDataFormatting) {
        parameter = PicamParameter_DisableDataFormatting;
    }
    else if (driverParameter == PICAM_EnableModulationOutputSignal) {
        parameter = PicamParameter_EnableModulationOutputSignal;
    }
    else if (driverParameter == PICAM_EnableNondestructiveReadout) {
        parameter = PicamParameter_EnableNondestructiveReadout;
    }
    else if (driverParameter == PICAM_EnableSensorWindowHeater) {
        parameter = PicamParameter_EnableSensorWindowHeater;
    }
    else if (driverParameter == PICAM_EnableSyncMaster) {
        parameter = PicamParameter_EnableSyncMaster;
    }
    else if (driverParameter == PICAM_ExactReadoutCountMax) {
        parameter = PicamParameter_ExactReadoutCountMaximum;
    }
    else if (driverParameter == ADAcquireTime) {
        parameter = PicamParameter_ExposureTime;
    }
    else if (driverParameter == PICAM_FrameRateCalc) {
        parameter = PicamParameter_FrameRateCalculation;
    }
    else if (driverParameter == NDArraySize) {
        parameter = PicamParameter_FrameSize;
    }
    else if (driverParameter == PICAM_FramesPerReadout) {
        parameter = PicamParameter_FramesPerReadout;
    }
    else if (driverParameter == PICAM_FrameStride) {
        parameter = PicamParameter_FrameStride;
    }
    else if (driverParameter == PICAM_FrameTrackingBitDepth) {
        parameter = PicamParameter_FrameTrackingBitDepth;
    }
    else if (driverParameter == PICAM_GateTracking) {
        parameter = PicamParameter_GateTracking;
    }
    else if (driverParameter == PICAM_GateTrackingBitDepth) {
        parameter = PicamParameter_GateTrackingBitDepth;
    }
    else if (driverParameter == PICAM_InvertOutputSignal) {
        parameter = PicamParameter_InvertOutputSignal;
    }
    else if (driverParameter == PICAM_KineticsWindowHeight) {
        parameter = PicamParameter_KineticsWindowHeight;
    }
    else if (driverParameter == PICAM_MaskedBottomMargin){
        parameter = PicamParameter_MaskedBottomMargin;
    }
    else if (driverParameter == PICAM_MaskedHeight){
        parameter = PicamParameter_MaskedHeight;
    }
    else if (driverParameter == PICAM_MaskedTopMargin){
        parameter = PicamParameter_MaskedTopMargin;
    }
    else if (driverParameter == PICAM_ModulationTracking) {
        parameter = PicamParameter_ModulationTracking;
    }
    else if (driverParameter == PICAM_ModulationTrackingBitDepth) {
        parameter = PicamParameter_ModulationTrackingBitDepth;
    }
    else if (driverParameter == PICAM_ModulationOutputSignalAmplitude) {
        parameter = PicamParameter_ModulationOutputSignalAmplitude;
    }
    else if (driverParameter == PICAM_ModulationOutputSignalFrequency) {
        parameter = PicamParameter_ModulationOutputSignalFrequency;
    }
    else if (driverParameter == PICAM_NormalizeOrientation) {
        parameter = PicamParameter_NormalizeOrientation;
    }
    else if (driverParameter == PICAM_NondestructiveReadoutPeriod) {
        parameter = PicamParameter_NondestructiveReadoutPeriod;
    }
    else if (driverParameter == PICAM_OnlineReadoutRateCalc) {
        parameter = PicamParameter_OnlineReadoutRateCalculation;
    }
    else if (driverParameter == PICAM_Orientation) {
        parameter = PicamParameter_Orientation;
    }
    else if (driverParameter == PICAM_OutputSignal) {
        parameter = PicamParameter_OutputSignal;
    }
    else if (driverParameter == PICAM_PhotonDetectionMode) {
        parameter = PicamParameter_PhotonDetectionMode;
    }
    else if (driverParameter == PICAM_PhotonDetectionThreshold) {
        parameter = PicamParameter_PhotonDetectionThreshold;
    }
    else if (driverParameter == PICAM_PixelBitDepth) {
        parameter = PicamParameter_PixelBitDepth;
    }
    else if (driverParameter == PICAM_PixelFormat) {
        parameter = PicamParameter_PixelFormat;
    }
    else if (driverParameter == PICAM_PixelGapHeight) {
        parameter = PicamParameter_PixelGapHeight;
    }
    else if (driverParameter == PICAM_PixelGapWidth) {
        parameter = PicamParameter_PixelGapWidth;
    }
    else if (driverParameter == PICAM_PixelHeight) {
        parameter = PicamParameter_PixelHeight;
    }
    else if (driverParameter == PICAM_PixelWidth) {
        parameter = PicamParameter_PixelWidth;
    }
    else if (driverParameter == PICAM_ReadoutCount) {
        parameter = PicamParameter_ReadoutCount;
    }
    else if (driverParameter == PICAM_ReadoutOrientation) {
        parameter = PicamParameter_ReadoutOrientation;
    }
    else if (driverParameter == PICAM_ReadoutRateCalc) {
        parameter = PicamParameter_ReadoutRateCalculation;
    }
    else if (driverParameter == PICAM_ReadoutStride) {
        parameter = PicamParameter_ReadoutStride;
    }
    else if (driverParameter == PICAM_SecondaryActiveHeight){
        parameter = PicamParameter_SecondaryActiveHeight;
    }
    else if (driverParameter == PICAM_SecondaryMaskedHeight){
        parameter = PicamParameter_SecondaryMaskedHeight;
    }
    else if (driverParameter == PICAM_SensorActiveBottomMargin){
        parameter = PicamParameter_SensorActiveBottomMargin;
    }
    else if (driverParameter == PICAM_SensorActiveHeight){
        parameter = PicamParameter_SensorActiveHeight;
    }
    else if (driverParameter == PICAM_SensorActiveLeftMargin){
        parameter = PicamParameter_SensorActiveLeftMargin;
    }
    else if (driverParameter == PICAM_SensorActiveRightMargin){
        parameter = PicamParameter_SensorActiveRightMargin;
    }
    else if (driverParameter == PICAM_SensorActiveTopMargin){
        parameter = PicamParameter_SensorActiveTopMargin;
    }
    else if (driverParameter == PICAM_SensorActiveWidth){
        parameter = PicamParameter_SensorActiveWidth;
    }
    else if (driverParameter == PICAM_SensorMaskedBottomMargin){
        parameter = PicamParameter_SensorMaskedBottomMargin;
    }
    else if (driverParameter == PICAM_SensorMaskedHeight){
        parameter = PicamParameter_SensorMaskedHeight;
    }
    else if (driverParameter == PICAM_SensorMaskedTopMargin){
        parameter = PicamParameter_SensorMaskedTopMargin;
    }
    else if (driverParameter == PICAM_SensorSecondaryActiveHeight){
        parameter = PicamParameter_SensorSecondaryActiveHeight;
    }
    else if (driverParameter == PICAM_SensorSecondaryMaskedHeight){
        parameter = PicamParameter_SensorSecondaryMaskedHeight;
    }
    else if (driverParameter == ADShutterCloseDelay){
        parameter = PicamParameter_ShutterClosingDelay;
    }
    else if (driverParameter == PICAM_ShutterDelayResolution){
        parameter = PicamParameter_ShutterDelayResolution;
    }
    else if (driverParameter == ADShutterOpenDelay){
        parameter = PicamParameter_ShutterOpeningDelay;
    }
    else if (driverParameter == PICAM_ShutterTimingMode){
        parameter = PicamParameter_ShutterTimingMode;
    }

    else if (driverParameter == ADTemperatureActual){
        parameter = PicamParameter_SensorTemperatureReading;
    }
    else if (driverParameter == ADTemperature){
        parameter = PicamParameter_SensorTemperatureSetPoint;
    }
    else if (driverParameter == PICAM_SensorTemperatureStatus){
        parameter = PicamParameter_SensorTemperatureStatus;
    }
    else if (driverParameter == PICAM_SensorType){
        parameter = PicamParameter_SensorType;
    }
    else if (driverParameter == PICAM_TimeStampBitDepth) {
        parameter = PicamParameter_TimeStampBitDepth;
    }
    else if (driverParameter == PICAM_TimeStampResolution) {
        parameter = PicamParameter_TimeStampResolution;
    }
    else if (driverParameter == PICAM_TimeStamps) {
        parameter = PicamParameter_TimeStamps;
    }
    else if (driverParameter == PICAM_TrackFrames) {
        parameter = PicamParameter_TrackFrames;
    }
    else if (driverParameter ==PICAM_ReadoutControlMode) {
        parameter = PicamParameter_ReadoutControlMode;
    }
    else if (driverParameter == PICAM_ShutterTimingMode) {
        parameter = PicamParameter_ShutterTimingMode;
    }
    else if (driverParameter == PICAM_SyncMaster2Delay) {
        parameter = PicamParameter_SyncMaster2Delay;
    }
    else if (driverParameter == PICAM_TriggerCoupling) {
        parameter = PicamParameter_TriggerCoupling;
    }
    else if (driverParameter == PICAM_TriggerDetermination) {
        parameter = PicamParameter_TriggerDetermination;
    }
    else if (driverParameter ==PICAM_TriggerFrequency) {
        parameter = PicamParameter_TriggerFrequency;
    }
    else if (driverParameter ==ADTriggerMode) {
        parameter = PicamParameter_TriggerResponse;
    }
    else if (driverParameter ==PICAM_TriggerSource) {
        parameter = PicamParameter_TriggerSource;
    }
    else if (driverParameter ==PICAM_TriggerTermination) {
        parameter = PicamParameter_TriggerTermination;
    }
    else if (driverParameter ==PICAM_TriggerThreshold) {
        parameter = PicamParameter_TriggerThreshold;
    }

    else {
        return PicamError_ParameterDoesNotExist;
    }
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: driverParameter: %d, picamParam: %d\n",
            driverName,
            functionName,
            driverParameter,
            parameter);
    return PicamError_None;
}

/**
 * Handler method for AcquisitionUpdated events.  Grab information
 * about acquired data, as necessary, and send a signal to a thread to
 * grab the data & process into NDArray.
 */
asynStatus ADPICam::piHandleAcquisitionUpdated(
        PicamHandle device,
        const PicamAvailableData *available,
        const PicamAcquisitionStatus *acqStatus)
{
    const char * functionName = "piHandleAcquisitionUpdated";
    int status = asynSuccess;

    dataLock.lock();
    acqStatusRunning = acqStatus->running;
    acqStatusErrors = acqStatus->errors;
    acqStatusReadoutRate = acqStatus->readout_rate;
    if ( (acqStatusErrors == PicamAcquisitionErrorsMask_None) &&
    		acqStatusRunning){
        acqAvailableInitialReadout = available->initial_readout;
        acqAvailableReadoutCount = available->readout_count;
        epicsEventSignal(piHandleNewImageEvent);
    }
    else {
        acqAvailableInitialReadout = NULL;
        acqAvailableReadoutCount = 0;
    }
    dataLock.unlock();

    return asynSuccess;
}

/**
 * Handler method for camera discovery events.  When new cameras become
 * available, or unavailable, move them on and off the lists as appropriate.
 */
asynStatus ADPICam::piHandleCameraDiscovery(const PicamCameraID *id,
        PicamHandle device, PicamDiscoveryAction action) {
    const char *functionName = "piHandleCameraDiscovery";
    PicamError error = PicamError_None;
    int status = asynSuccess;
    const char* modelString;

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s Enter\n",
            driverName,
            functionName);

    piLoadAvailableCameraIDs();
    piUpdateAvailableCamerasList();
    switch (action) {
    case PicamDiscoveryAction_Found:
        Picam_GetEnumerationString(PicamEnumeratedType_Model, id->model,
                &modelString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Camera Found: %s\n",
                modelString);
        Picam_DestroyString(modelString);
        if (device != NULL) {
            PicamHandle discoveredModel;
            PicamAdvanced_GetCameraModel(device, &discoveredModel);
            printf(" discovered %s, current, %s\n", discoveredModel,
                    currentCameraHandle);
            if (discoveredModel == currentCameraHandle) {
                piSetSelectedCamera(pasynUserSelf, selectedCameraIndex);
            }

        }
        break;
    case PicamDiscoveryAction_Lost:
        Picam_GetEnumerationString(PicamEnumeratedType_Model, id->model,
                &modelString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Camera Lost: %s\n",
                modelString);
        Picam_DestroyString(modelString);

        if (device != NULL) {
            PicamHandle discoveredModel;
            PicamAdvanced_GetCameraModel(device, &discoveredModel);
            printf(" discovered %s, current, %s", discoveredModel,
                    currentCameraHandle);
            if (discoveredModel == currentCameraHandle) {
                setStringParam(PICAM_CameraInterface, notAvailable);
                setStringParam(PICAM_SensorName, notAvailable);
                setStringParam(PICAM_SerialNumber, notAvailable);
                setStringParam(PICAM_FirmwareRevision, notAvailable);
                callParamCallbacks();
            }
        }
        break;
    default:
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "Unexpected discovery action%d", action);
    }
    if (device == NULL) {
        Picam_GetEnumerationString(PicamEnumeratedType_Model, id->model,
                &modelString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "No device found with camera\n");
        Picam_DestroyString(modelString);
    } else {
        Picam_GetEnumerationString(PicamEnumeratedType_Model, id->model,
                &modelString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "Device found with camera\n");

        Picam_DestroyString(modelString);

    }

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s Exit\n",
            driverName,
            functionName);

    return (asynStatus) status;
}

/**
 * Handler method called by piParameterFloatingPointValueChanged callback method
 * Makes necessary since the parameter has changed.  Primarily will update the
 * Readback value for many parameters.
 */
asynStatus ADPICam::piHandleParameterFloatingPointValueChanged(
        PicamHandle camera, PicamParameter parameter, piflt value) {
    const char *functionName = "piHandleParameterFloatingPointValueChanged";
    PicamError error = PicamError_None;
    int status = asynSuccess;
    const pichar *parameterString;
    int driverParameter;

    driverParameter = piLookupDriverParameter(parameter);
    // Handle special cases
    if (parameter == PicamParameter_AdcSpeed) {
        const PicamCollectionConstraint *speedConstraint;
        piflt fltVal;
        error = Picam_GetParameterCollectionConstraint(currentCameraHandle,
                PicamParameter_AdcSpeed, PicamConstraintCategory_Capable,
                &speedConstraint);
        error = Picam_GetParameterFloatingPointValue(currentCameraHandle,
                PicamParameter_AdcSpeed, &fltVal);
        printf("setting value for ADCspeed % f\n", fltVal);
        for (int ii = 0; ii < speedConstraint->values_count; ii++) {
            if (speedConstraint->values_array[ii] == fltVal) {
                setIntegerParam(driverParameter, ii);
            }
        }
    //Handle the cases where simple translation between PICAM and areaDetector
    //is possible
    } else if (driverParameter >= 0) {
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
                &parameterString);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s Setting PICAM parameter %s to driverParameter %d, "
        		"value %f\n",
                driverName, functionName, parameterString, driverParameter,
                value);
        Picam_DestroyString(parameterString);
        setDoubleParam(driverParameter, value);
        // Notify that handling a parameter is about to fall on the floor
        // unhandled
    } else {
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
                &parameterString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Parameter %s floating point value Changed to %f.  "
                        "This change is not handled.\n", driverName,
                functionName, parameterString, value);
        Picam_DestroyString(parameterString);
        switch (parameter) {
        case PicamParameter_ExposureTime:
            setDoubleParam(ADAcquireTime, value);
        }
    }
    callParamCallbacks();
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s Exit\n", driverName,
            functionName);

    return (asynStatus) status;
}

/**
 * Handler method called by piParameterIntegerValueChanged callback method
 * Makes necessary since the parameter has changed.  Primarily will update the
 * Readback value for many parameters.
 */
asynStatus ADPICam::piHandleParameterIntegerValueChanged(PicamHandle camera,
        PicamParameter parameter, piint value) {
    const char *functionName = "piHandleParameterIntegerValueChanged";
    PicamError error = PicamError_None;
    int status = asynSuccess;
    const char *parameterString;
    int driverParameter

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s Enter\n",
            driverName,
            functionName);

    driverParameter = piLookupDriverParameter(parameter);
    //Handle parameters that are as easy as translating between PICAM and
    //areaDetector
    if (driverParameter >= 0) {
        setIntegerParam(driverParameter, value);
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
                &parameterString);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s Setting PICAM parameter %s to driverParameter %d, "
                "value %f\n",
                driverName, functionName, parameterString, driverParameter,
                value);
        Picam_DestroyString(parameterString);
        // Pass along to method that lets you know that a parameter change is
        // about to fall on the ground unhandled.
    } else {
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
                &parameterString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Parameter %s integer value Changed %d."
                        " This change was unhandled.\n", driverName,
                functionName, parameterString, (int )value);
        Picam_DestroyString(parameterString);
    }
    callParamCallbacks();

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s Exit\n",
            driverName,
            functionName);

    return (asynStatus) status;
}

/**
 * Handler method called by piParameterLargeIntegerValueChanged callback method
 * Makes necessary since the parameter has changed.  Primarily will update the
 * Readback value for many parameters.
 */
asynStatus ADPICam::piHandleParameterLargeIntegerValueChanged(
        PicamHandle camera,
        PicamParameter parameter,
        pi64s value) {
    const char *functionName = "piHandleParameterLargeIntegerValueChanged";
    PicamError error = PicamError_None;
    int status = asynSuccess;
    const char *parameterString;
    int driverParameter

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s Enter\n",
            driverName,
            functionName);
    //Handle parameters that are a simple lookup between areaDetector and PICAM
    driverParameter = piLookupDriverParameter(parameter);
    if (driverParameter >= 0) {
        long lValue = (long) value;
        setIntegerParam(driverParameter, lValue);
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
                &parameterString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Setting PICAM parameter %s to driverParameter %d, "
                "value %d long value %d\n",
                driverName, functionName, parameterString, driverParameter,
                value, lValue);
        Picam_DestroyString(parameterString);
    // Notify that handling a parameter is about to fall on the floor unhandled
    } else {
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
                &parameterString);
        printf("%s:%s Parameter %s large integer value Changed %d\n",
                driverName, functionName, parameterString, value);
        Picam_DestroyString(parameterString);
    }

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s Exit\n", driverName,
            functionName);
    return (asynStatus) status;
}

/**
 * Handle case when a PicamModulations value has changed.  Called by
 * piParameterModulationsValueChanged.
 */
asynStatus ADPICam::piHandleParameterModulationsValueChanged(PicamHandle camera,
        PicamParameter parameter, const PicamModulations *value) {
    const char *functionName = "piHandleParameterModulationsValueChanged";
    PicamError error = PicamError_None;
    int status = asynSuccess;
    const char *parameterString;
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s Enter\n", driverName,
            functionName);
    Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
            &parameterString);
    printf("parameter %s Modulations value Changed to %f", parameterString,
            value);
    Picam_DestroyString(parameterString);

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s Exit\n", driverName,
            functionName);

    return (asynStatus) status;
}

/**
 * Handle case when a PicamPulse value has changed.  Called by
 * piParameterPulseValueChanged.
 */
asynStatus ADPICam::piHandleParameterPulseValueChanged(PicamHandle camera,
        PicamParameter parameter, const PicamPulse *value) {
    const char *functionName = "piHandleParameterPulseValueChanged";
    PicamError error = PicamError_None;
    int status = asynSuccess;
    const char *parameterString;
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s Enter\n", driverName,
            functionName);
    Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
            &parameterString);
    printf("parameter %s Pulse value Changed to %f\n", parameterString, value);
    Picam_DestroyString(parameterString);

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s Exit\n", driverName,
            functionName);

    return (asynStatus) status;
}

/**
 * Handler method called by piParameterRelevanceChanged callback method
 * Sets the relevence of a parameter based on changes in parameters.
 */
asynStatus ADPICam::piHandleParameterRelevanceChanged(PicamHandle camera,
        PicamParameter parameter, pibln relevant) {
    const char *functionName = "piHandleParameterRelevanceChanged";
    PicamError error = PicamError_None;
    int status = asynSuccess;
    PicamConstraintType constraintType;
    PicamValueType valueType;
    int driverParameter;

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s Enter",
            driverName,
            functionName);

    status = piSetParameterRelevance(pasynUserSelf, parameter, (int) relevant);
    if (relevant != 0) {
    	Picam_GetParameterConstraintType(currentCameraHandle, parameter, &constraintType);
    	Picam_GetParameterValueType(currentCameraHandle, parameter, &valueType);
    	driverParameter = piLookupDriverParameter(parameter);
    	if ((driverParameter > 0) &&
    			((constraintType == PicamConstraintType_Collection) ||
    					(valueType == PicamValueType_Enumeration))) {
    		piUpdateParameterListValues(parameter, driverParameter);
    	}
    }
    return (asynStatus) status;
}

/**
 * Handle the case that an ROI value has changed.
 */
asynStatus ADPICam::piHandleParameterRoisValueChanged(PicamHandle camera,
        PicamParameter parameter, const PicamRois *value) {
    const char *functionName = "piHandleParameterRoisValueChanged";
    PicamError error = PicamError_None;
    int status = asynSuccess;
    const char *parameterString;
    Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
            &parameterString);
    printf("parameter %s Rois value Changed\n", parameterString);
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "\n----- minX = %d\n----- minY = %d\n"
                    "----- sizeX = %d\n----- sizeY = %d\n"
                    "----- binX = %d\n----- binY = %d\n", value->roi_array[0].x,
            value->roi_array[0].y, value->roi_array[0].width,
            value->roi_array[0].height, value->roi_array[0].x_binning,
            value->roi_array[0].y_binning);
    setIntegerParam(NDArraySizeX, value->roi_array[0].width/
    		value->roi_array[0].x_binning);
    setIntegerParam(NDArraySizeY, value->roi_array[0].height/
    		value->roi_array[0].y_binning);
    Picam_DestroyString(parameterString);

    callParamCallbacks();
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s Exit\n", driverName,
            functionName);



    return (asynStatus) status;
}

/**
 * Callback to Handle when a FloatingPoint value changes.  Hand off to
 * method piHandleParameterFloatingPointValue of the stored class instance
 */
PicamError PIL_CALL ADPICam::piParameterFloatingPointValueChanged(
        PicamHandle camera, PicamParameter parameter, piflt value) {
    int status = asynSuccess;
    PicamError error = PicamError_None;
    const char *functionName = "piParameterFloatingPointValueChanged";

    status = ADPICam_Instance->piHandleParameterFloatingPointValueChanged(
            camera, parameter, value);

    return error;
}

/**
 * Callback method to handle when an Integer Value Changes.  Hand off to the
 * method piHandleParameterIntergerValueChanged of the stored class instance.
 */
PicamError PIL_CALL ADPICam::piParameterIntegerValueChanged(PicamHandle camera,
        PicamParameter parameter, piint value) {
    int status = asynSuccess;
    PicamError error = PicamError_None;
    const char *functionName = "piParameterIntegerValueChanged";

    status = ADPICam_Instance->piHandleParameterIntegerValueChanged(camera,
            parameter, value);

    return error;
}

/**
 * Callback to Handle when a LargeInteger value changes.  Hand of to the method
 * piHandleParameterLargeIntergerValueChanged of the stored class instance.
 */
PicamError PIL_CALL ADPICam::piParameterLargeIntegerValueChanged(
        PicamHandle camera, PicamParameter parameter, pi64s value) {
    int status = asynSuccess;
    PicamError error = PicamError_None;
    const char *functionName = "piParameterLargeIntegerValueChanged";

    status = ADPICam_Instance->piHandleParameterLargeIntegerValueChanged(camera,
            parameter, value);

    return error;
}

/**
 * Callback to Handle when a PicamModulations value changes.  Hand off to the
 * method picamHandleModulationValueChanged of the stored class instance.
 */
PicamError PIL_CALL ADPICam::piParameterModulationsValueChanged(
        PicamHandle camera, PicamParameter parameter,
        const PicamModulations *value) {
    int status = asynSuccess;
    PicamError error = PicamError_None;
    const char *functionName = "piParameterModulationsValueChanged";

    status = ADPICam_Instance->piHandleParameterModulationsValueChanged(camera,
            parameter, value);

    return error;
}

/**
 * Callback to Handle when a PicamPulse value changes.  Calls method
 * piHandleParameterPulseValueChanged of the stored class instance.
 */
PicamError PIL_CALL ADPICam::piParameterPulseValueChanged(PicamHandle camera,
        PicamParameter parameter, const PicamPulse *value) {
    int status = asynSuccess;
    PicamError error = PicamError_None;
    const char *functionName = "piParameterPulseValueChanged";

    status = ADPICam_Instance->piHandleParameterPulseValueChanged(camera,
            parameter, value);

    return error;
}

/**
 * Callback event to catch when a parameter's relevance has changed.  Calls
 * method piHandleParameterRelevanceChanged of stored class instance.
 */
PicamError PIL_CALL ADPICam::piParameterRelevanceChanged(PicamHandle camera,
        PicamParameter parameter, pibln relevent) {
    int status = asynSuccess;
    PicamError error = PicamError_None;

    status = ADPICam_Instance->piHandleParameterRelevanceChanged(camera,
            parameter, relevent);

    return error;
}

/**
 * Callback to Handle when a Roi value changes.  Calls method
 * piHandleParameterRoisValueChanged of the stored class instance.
 */
PicamError PIL_CALL ADPICam::piParameterRoisValueChanged(PicamHandle camera,
        PicamParameter parameter, const PicamRois *value) {
    int status = asynSuccess;
    PicamError error = PicamError_None;
    const char *functionName = "piParameterRoisValueChanged";

    status = ADPICam_Instance->piHandleParameterRoisValueChanged(camera,
            parameter, value);

    return error;
}

/**
 * Print the Rois constraint information.
 */
asynStatus ADPICam::piPrintRoisConstraints() {
    const char *functionName = "piPrintRoisConstraints";
    PicamError error = PicamError_None;
    const PicamRoisConstraint *roisConstraints;
    const char *errorString;
    int status = asynSuccess;

    error = Picam_GetParameterRoisConstraint(currentCameraHandle,
            PicamParameter_Rois, PicamConstraintCategory_Capable,
            &roisConstraints);
    if (error != PicamError_None) {
        Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                &errorString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Error retrieving rois constraints %s\n", driverName,
                functionName, errorString);
        Picam_DestroyString(errorString);
        return asynError;
    }

    return (asynStatus)status;
}

/**
 *Register callbacks for constraint changes.
 */
asynStatus ADPICam::piRegisterConstraintChangeWatch(PicamHandle cameraHandle) {
    int status = asynSuccess;
    piint parameterCount;
    const PicamParameter *parameterList;
    PicamError error;

    error = Picam_GetParameters(currentCameraHandle, &parameterList,
            &parameterCount);
    if (error != PicamError_None) {
        //TODO
    }
    for (int ii = 0; ii < parameterCount; ii++) {
        error = PicamAdvanced_UnregisterForIsRelevantChanged(cameraHandle,
                parameterList[ii], piParameterRelevanceChanged);
    }
    return (asynStatus) status;
}

/**
 * Register to watch to changes in parameter relevance
 */
asynStatus ADPICam::piRegisterRelevantWatch(PicamHandle cameraHandle) {
    int status = asynSuccess;
    piint parameterCount;
    const PicamParameter *parameterList;
    PicamError error;

    error = Picam_GetParameters(currentCameraHandle, &parameterList,
            &parameterCount);
    if (error != PicamError_None) {
        //TODO
    }
    for (int ii = 0; ii < parameterCount; ii++) {
        error = PicamAdvanced_RegisterForIsRelevantChanged(cameraHandle,
                parameterList[ii], piParameterRelevanceChanged);
    }
    return (asynStatus) status;
}

/**
 * Register to watch for changes in a parameter's value
 */
asynStatus ADPICam::piRegisterValueChangeWatch(PicamHandle cameraHandle) {
    int status = asynSuccess;
    piint parameterCount;
    const PicamParameter *parameterList;
    PicamValueType valueType;
    PicamError error;
    const char *functionName = "piRegisterValueChangeWatch";
    pibln doesParamExist;

    error = Picam_GetParameters(currentCameraHandle, &parameterList,
            &parameterCount);
    if (error != PicamError_None) {
        //TODO
    }
    for (int ii = 0; ii < parameterCount; ii++) {
    	Picam_DoesParameterExist(currentCameraHandle,
    			parameterList[ii],
				&doesParamExist);
    	if (doesParamExist) {
			error = Picam_GetParameterValueType(cameraHandle, parameterList[ii],
					&valueType);
			if (error != PicamError_None) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s \n", driverName,
						functionName);
				return asynError;
			}
			switch (valueType) {
			case PicamValueType_Integer:
			case PicamValueType_Boolean:
			case PicamValueType_Enumeration:
				error = PicamAdvanced_RegisterForIntegerValueChanged(cameraHandle,
						parameterList[ii], piParameterIntegerValueChanged);
				break;
			case PicamValueType_LargeInteger:
				error = PicamAdvanced_RegisterForLargeIntegerValueChanged(
						cameraHandle, parameterList[ii],
						piParameterLargeIntegerValueChanged);
				break;
			case PicamValueType_FloatingPoint:
				error = PicamAdvanced_RegisterForFloatingPointValueChanged(
						cameraHandle, parameterList[ii],
						piParameterFloatingPointValueChanged);
				break;
			case PicamValueType_Rois:
				printf("Registering ROIS value changed\n");
				error = PicamAdvanced_RegisterForRoisValueChanged(cameraHandle,
						parameterList[ii], piParameterRoisValueChanged);
				break;
			case PicamValueType_Pulse:
				error = PicamAdvanced_RegisterForPulseValueChanged(cameraHandle,
						parameterList[ii], piParameterPulseValueChanged);
				break;
			case PicamValueType_Modulations:
				error = PicamAdvanced_RegisterForModulationsValueChanged(
						cameraHandle, parameterList[ii],
						piParameterModulationsValueChanged);
				break;
			default: {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
						"%s:%s Unexpected valueType %s", driverName, functionName,
						valueType);
				return asynError;
			}
				break;
			}
    	}
    }
    return (asynStatus) status;
}

/**
 * Set the value stored in a parameter existance PV
 */
asynStatus ADPICam::piSetParameterExists(asynUser *pasynUser,
        PicamParameter parameter, int exists) {
    int status = asynSuccess;
    int driverParameter = -1;
    static const char *functionName = "piSetParameterExists";
    const pichar* string;

    switch (parameter) {
    case PicamParameter_Accumulations:
        driverParameter = PICAM_AccumulationsExists;
        break;
    case PicamParameter_ActiveBottomMargin:
        driverParameter = PICAM_ActiveBottomMarginExists;
        break;
    case PicamParameter_ActiveHeight:
        driverParameter = PICAM_ActiveHeightExists;
        break;
    case PicamParameter_ActiveLeftMargin:
        driverParameter = PICAM_ActiveLeftMarginExists;
        break;
    case PicamParameter_ActiveRightMargin:
        driverParameter = PICAM_ActiveRightMarginExists;
        break;
    case PicamParameter_ActiveTopMargin:
        driverParameter = PICAM_ActiveTopMarginExists;
        break;
    case PicamParameter_ActiveWidth:
        driverParameter = PICAM_ActiveWidthExists;
        break;
    case PicamParameter_AdcAnalogGain:
        driverParameter = PICAM_AdcAnalogGainExists;
        break;
    case PicamParameter_AdcBitDepth:
        driverParameter = PICAM_AdcBitDepthExists;
        break;
    case PicamParameter_AdcEMGain:
        driverParameter = PICAM_AdcEMGainExists;
        break;
    case PicamParameter_AdcQuality:
        driverParameter = PICAM_AdcQualityExists;
        break;
    case PicamParameter_AdcSpeed:
        driverParameter = PICAM_AdcSpeedExists;
        break;
    case PicamParameter_AuxOutput:
        driverParameter = PICAM_AuxOutputExists;
        break;
    case PicamParameter_BracketGating:
        driverParameter = PICAM_BracketGatingExists;
        break;
    case PicamParameter_CcdCharacteristics:
        driverParameter = PICAM_CcdCharacteristicsExists;
        break;
    case PicamParameter_CleanBeforeExposure:
        driverParameter = PICAM_CleanBeforeExposure;
        break;
    case PicamParameter_CleanCycleCount:
        driverParameter = PICAM_CleanCycleCountExists;
        break;
    case PicamParameter_CleanCycleHeight:
        driverParameter = PICAM_CleanCycleHeightExists;
        break;
    case PicamParameter_CleanSectionFinalHeight:
        driverParameter = PICAM_CleanSectionFinalHeightExists;
        break;
    case PicamParameter_CleanSectionFinalHeightCount:
        driverParameter = PICAM_CleanSectionFinalHeightCountExists;
        break;
    case PicamParameter_CleanSerialRegister:
        driverParameter = PICAM_CleanSerialRegisterExists;
        break;
    case PicamParameter_CleanUntilTrigger:
        driverParameter = PICAM_CleanUntilTriggerExists;
        break;
    case PicamParameter_CorrectPixelBias:
        driverParameter = PICAM_CorrectPixelBiasExists;
        break;
    case PicamParameter_CustomModulationSequence:
        driverParameter = PICAM_CustomModulationSequenceExists;
        break;
    case PicamParameter_DifEndingGate:
        driverParameter = PICAM_DifEndingGateExists;
        break;
    case PicamParameter_DifStartingGate:
        driverParameter = PICAM_DifStartingGateExists;
        break;
    case PicamParameter_DisableCoolingFan:
        driverParameter = PICAM_DisableCoolingFanExists;
        break;
    case PicamParameter_DisableDataFormatting:
        driverParameter = PICAM_DisableDataFormattingExists;
        break;
    case PicamParameter_EMIccdGain:
        driverParameter = PICAM_EMIccdGainExists;
        break;
    case PicamParameter_EMIccdGainControlMode:
        driverParameter = PICAM_EMIccdGainControlModeExists;
        break;
    case PicamParameter_EnableIntensifier:
        driverParameter = PICAM_EnableIntensifierExists;
        break;
    case PicamParameter_EnableModulation:
        driverParameter = PICAM_EnableModulationExists;
        break;
    case PicamParameter_EnableModulationOutputSignal:
        driverParameter = PICAM_EnableModulationOutputSignalExists;
        break;
    case PicamParameter_EnableNondestructiveReadout:
        driverParameter = PICAM_EnableNondestructiveReadoutExists;
        break;
    case PicamParameter_EnableSensorWindowHeater:
        driverParameter = PICAM_EnableSensorWindowHeaterExists;
        break;
    case PicamParameter_EnableSyncMaster:
        driverParameter = PICAM_EnableSyncMasterExists;
        break;
    case PicamParameter_ExactReadoutCountMaximum:
        driverParameter = PICAM_ExactReadoutCountMaximumExists;
        break;
    case PicamParameter_ExposureTime:
        driverParameter = PICAM_ExposureTimeExists;
        break;
    case PicamParameter_FrameRateCalculation:
        driverParameter = PICAM_FrameRateCalculationExists;
        break;
    case PicamParameter_FrameSize:
        driverParameter = PICAM_FrameSizeExists;
        break;
    case PicamParameter_FramesPerReadout:
        driverParameter = PICAM_FramesPerReadoutExists;
        break;
    case PicamParameter_FrameStride:
        driverParameter = PICAM_FrameStrideExists;
        break;
    case PicamParameter_FrameTrackingBitDepth:
        driverParameter = PICAM_FrameTrackingBitDepthExists;
        break;
    case PicamParameter_GateTracking:
        driverParameter = PICAM_GateTrackingExists;
        break;
    case PicamParameter_GateTrackingBitDepth:
        driverParameter = PICAM_GateTrackingBitDepthExists;
        break;
    case PicamParameter_GatingMode:
        driverParameter = PICAM_GatingModeExists;
        break;
    case PicamParameter_GatingSpeed:
        driverParameter = PICAM_GatingSpeedExists;
        break;
    case PicamParameter_IntensifierDiameter:
        driverParameter = PICAM_IntensifierDiameterExists;
        break;
    case PicamParameter_IntensifierGain:
        driverParameter = PICAM_IntensifierGainExists;
        break;
    case PicamParameter_IntensifierOptions:
        driverParameter = PICAM_IntensifierOptionsExists;
        break;
    case PicamParameter_IntensifierStatus:
        driverParameter = PICAM_IntensifierStatusExists;
        break;
    case PicamParameter_InvertOutputSignal:
        driverParameter = PICAM_InvertOutputSignalExists;
        break;
    case PicamParameter_KineticsWindowHeight:
        driverParameter = PICAM_KineticsWindowHeightExists;
        break;
    case PicamParameter_MaskedBottomMargin:
        driverParameter = PICAM_MaskedBottomMarginExists;
        break;
    case PicamParameter_MaskedHeight:
        driverParameter = PICAM_MaskedHeightExists;
        break;
    case PicamParameter_MaskedTopMargin:
        driverParameter = PICAM_MaskedTopMarginExists;
        break;
    case PicamParameter_ModulationDuration:
        driverParameter = PICAM_ModulationDurationExists;
        break;
    case PicamParameter_ModulationFrequency:
        driverParameter = PICAM_ModulationFrequencyExists;
        break;
    case PicamParameter_ModulationOutputSignalAmplitude:
        driverParameter = PICAM_EnableModulationOutputSignalAmplitudeExists;
        break;
    case PicamParameter_ModulationOutputSignalFrequency:
        driverParameter = PICAM_EnableModulationOutputSignalFrequencyExists;
        break;
    case PicamParameter_ModulationTracking:
        driverParameter = PICAM_ModulationTrackingExists;
        break;
    case PicamParameter_ModulationTrackingBitDepth:
        driverParameter = PICAM_ModulationTrackingBitDepthExists;
        break;
    case PicamParameter_NondestructiveReadoutPeriod:
        driverParameter = PICAM_NondestructiveReadoutPeriodExists;
        break;
    case PicamParameter_NormalizeOrientation:
        driverParameter = PICAM_NormalizeOrientationExists;
        break;
    case PicamParameter_OnlineReadoutRateCalculation:
        driverParameter = PICAM_OnlineReadoutRateCalculationExists;
        break;
    case PicamParameter_Orientation:
        driverParameter = PICAM_OrientationExists;
        break;
    case PicamParameter_OutputSignal:
        driverParameter = PICAM_OutputSignalExists;
        break;
    case PicamParameter_PhosphorDecayDelay:
        driverParameter = PICAM_PhosphorDecayDelayExists;
        break;
    case PicamParameter_PhosphorDecayDelayResolution:
        driverParameter = PICAM_PhosphorDecayDelayResolutionExists;
        break;
    case PicamParameter_PhosphorType:
        driverParameter = PICAM_PhosphorTypeExists;
        break;
    case PicamParameter_PhotocathodeSensitivity:
        driverParameter = PICAM_PhotocathodeSensitivityExists;
        break;
    case PicamParameter_PhotonDetectionMode:
        driverParameter = PICAM_PhotonDetectionModeExists;
        break;
    case PicamParameter_PhotonDetectionThreshold:
        driverParameter = PICAM_PhotonDetectionThresholdExists;
        break;
    case PicamParameter_PixelBitDepth:
        driverParameter = PICAM_PixelBitDepthExists;
        break;
    case PicamParameter_PixelFormat:
        driverParameter = PICAM_PixelFormatExists;
        break;
    case PicamParameter_PixelGapHeight:
        driverParameter = PICAM_PixelGapHeightExists;
        break;
    case PicamParameter_PixelGapWidth:
        driverParameter = PICAM_PixelGapWidthExists;
        break;
    case PicamParameter_PixelHeight:
        driverParameter = PICAM_PixelHeightExists;
        break;
    case PicamParameter_PixelWidth:
        driverParameter = PICAM_PixelWidthExists;
        break;
    case PicamParameter_ReadoutControlMode:
        driverParameter = PICAM_ReadoutControlModeExists;
        break;
    case PicamParameter_ReadoutCount:
        driverParameter = PICAM_ReadoutCountExists;
        break;
    case PicamParameter_ReadoutOrientation:
        driverParameter = PICAM_ReadoutOrientationExists;
        break;
    case PicamParameter_ReadoutPortCount:
        driverParameter = PICAM_ReadoutPortCountExists;
        break;
    case PicamParameter_ReadoutRateCalculation:
        driverParameter = PICAM_ReadoutRateCalculationExists;
        break;
    case PicamParameter_ReadoutStride:
        driverParameter = PICAM_ReadoutStrideExists;
        break;
    case PicamParameter_ReadoutTimeCalculation:
        driverParameter = PICAM_ReadoutTimeCalculationExists;
        break;
    case PicamParameter_RepetitiveGate:
        driverParameter = PICAM_RepetitiveGateExists;
        break;
    case PicamParameter_RepetitiveModulationPhase:
        driverParameter = PICAM_RepetitiveModulationPhaseExists;
        break;
    case PicamParameter_Rois:
        driverParameter = PICAM_RoisExists;
        break;
    case PicamParameter_SecondaryActiveHeight:
        driverParameter = PICAM_SecondaryActiveHeightExists;
        break;
    case PicamParameter_SecondaryMaskedHeight:
        driverParameter = PICAM_SecondaryMaskedHeightExists;
        break;
    case PicamParameter_SensorActiveBottomMargin:
        driverParameter = PICAM_SensorActiveBottomMarginExists;
        break;
    case PicamParameter_SensorActiveHeight:
        driverParameter = PICAM_SensorActiveHeightExists;
        break;
    case PicamParameter_SensorActiveLeftMargin:
        driverParameter = PICAM_SensorActiveLeftMarginExists;
        break;
    case PicamParameter_SensorActiveRightMargin:
        driverParameter = PICAM_SensorActiveRightMarginExists;
        break;
    case PicamParameter_SensorActiveTopMargin:
        driverParameter = PICAM_SensorActiveTopMarginExists;
        break;
    case PicamParameter_SensorActiveWidth:
        driverParameter = PICAM_SensorActiveWidthExists;
        break;
    case PicamParameter_SensorMaskedBottomMargin:
        driverParameter = PICAM_SensorMaskedBottomMarginExists;
        break;
    case PicamParameter_SensorMaskedHeight:
        driverParameter = PICAM_SensorMaskedHeightExists;
        break;
    case PicamParameter_SensorMaskedTopMargin:
        driverParameter = PICAM_SensorMaskedTopMarginExists;
        break;
    case PicamParameter_SensorSecondaryActiveHeight:
        driverParameter = PICAM_SensorSecondaryActiveHeightExists;
        break;
    case PicamParameter_SensorSecondaryMaskedHeight:
        driverParameter = PICAM_SensorSecondaryMaskedHeightExists;
        break;
    case PicamParameter_SensorTemperatureReading:
        driverParameter = PICAM_SensorTemperatureReadingExists;
        break;
    case PicamParameter_SensorTemperatureSetPoint:
        driverParameter = PICAM_SensorTemperatureSetPointExists;
        break;
    case PicamParameter_SensorTemperatureStatus:
        driverParameter = PICAM_SensorTemperatureStatusExists;
        break;
    case PicamParameter_SensorType:
        driverParameter = PICAM_SensorTypeExists;
        break;
    case PicamParameter_SequentialEndingGate:
        driverParameter = PICAM_SequentialEndingGateExists;
        break;
    case PicamParameter_SequentialEndingModulationPhase:
        driverParameter = PICAM_SequentialEndingModulationPhaseExists;
        break;
    case PicamParameter_SequentialGateStepCount:
        driverParameter = PICAM_SequentialGateStepCountExists;
        break;
    case PicamParameter_SequentialGateStepIterations:
        driverParameter = PICAM_SequentialGateStepIterationsExists;
        break;
    case PicamParameter_SequentialStartingGate:
        driverParameter = PICAM_SequentialStartingGateExists;
        break;
    case PicamParameter_SequentialStartingModulationPhase:
        driverParameter = PICAM_SequentialStartingModulationPhaseExists;
        break;
    case PicamParameter_ShutterClosingDelay:
        driverParameter = PICAM_ShutterClosingDelayExists;
        break;
    case PicamParameter_ShutterDelayResolution:
        driverParameter = PICAM_ShutterDelayResolutionExists;
        break;
    case PicamParameter_ShutterOpeningDelay:
        driverParameter = PICAM_ShutterOpeningDelayExists;
        break;
    case PicamParameter_ShutterTimingMode:
        driverParameter = PICAM_ShutterTimingModeExists;
        break;
    case PicamParameter_SyncMaster2Delay:
        driverParameter = PICAM_SyncMaster2DelayExists;
        break;
    case PicamParameter_TimeStampBitDepth:
        driverParameter = PICAM_TimeStampBitDepthExists;
        break;
    case PicamParameter_TimeStampResolution:
        driverParameter = PICAM_TimeStampResolutionExists;
        break;
    case PicamParameter_TimeStamps:
        driverParameter = PICAM_TimeStampsExists;
        break;
    case PicamParameter_TrackFrames:
        driverParameter = PICAM_TrackFramesExists;
        break;
    case PicamParameter_TriggerCoupling:
        driverParameter = PICAM_TriggerCouplingExists;
        break;
    case PicamParameter_TriggerDetermination:
        driverParameter = PICAM_TriggerDeterminationExists;
        break;
    case PicamParameter_TriggerFrequency:
        driverParameter = PICAM_TriggerFrequencyExists;
        break;
    case PicamParameter_TriggerResponse:
        driverParameter = PICAM_TriggerResponseExists;
        break;
    case PicamParameter_TriggerSource:
        driverParameter = PICAM_TriggerSourceExists;
        break;
    case PicamParameter_TriggerTermination:
        driverParameter = PICAM_TriggerTerminationExists;
        break;
    case PicamParameter_TriggerThreshold:
        driverParameter = PICAM_TriggerThresholdExists;
        break;
    case PicamParameter_VerticalShiftRate:
        driverParameter = PICAM_VerticalShiftRateExists;
        break;
    default:
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
                &string);
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "---- Can't find parameter %s\n",
                string);
        Picam_DestroyString(string);
        break;
    }
    setIntegerParam(driverParameter, exists);
    return (asynStatus) status;
}

/**
 * Set the value stored in a parameter relevance PV
 */
asynStatus ADPICam::piSetParameterRelevance(asynUser *pasynUser,
        PicamParameter parameter, int relevence) {
    int status = asynSuccess;
    int driverParameter = -1;
    static const char *functionName = "piSetSelectedCamera";
    const pichar* string;

    switch (parameter) {
    case PicamParameter_Accumulations:
        driverParameter = PICAM_AccumulationsRelevant;
        break;
    case PicamParameter_ActiveBottomMargin:
        driverParameter = PICAM_ActiveBottomMarginRelevant;
        break;
    case PicamParameter_ActiveHeight:
        driverParameter = PICAM_ActiveHeightRelevant;
        break;
    case PicamParameter_ActiveLeftMargin:
        driverParameter = PICAM_ActiveLeftMarginRelevant;
        break;
    case PicamParameter_ActiveRightMargin:
        driverParameter = PICAM_ActiveRightMarginRelevant;
        break;
    case PicamParameter_ActiveTopMargin:
        driverParameter = PICAM_ActiveTopMarginRelevant;
        break;
    case PicamParameter_ActiveWidth:
        driverParameter = PICAM_ActiveWidthRelevant;
        break;
    case PicamParameter_AdcAnalogGain:
        driverParameter = PICAM_AdcAnalogGainRelevant;
        break;
    case PicamParameter_AdcBitDepth:
        driverParameter = PICAM_AdcBitDepthRelevant;
        break;
    case PicamParameter_AdcEMGain:
        driverParameter = PICAM_AdcEMGainRelevant;
        break;
    case PicamParameter_AdcQuality:
        driverParameter = PICAM_AdcQualityRelevant;
        break;
    case PicamParameter_AdcSpeed:
        driverParameter = PICAM_AdcSpeedRelevant;
        break;
    case PicamParameter_AuxOutput:
        driverParameter = PICAM_AuxOutputRelevant;
        break;
    case PicamParameter_BracketGating:
        driverParameter = PICAM_BracketGatingRelevant;
        break;
    case PicamParameter_CcdCharacteristics:
        driverParameter = PICAM_CcdCharacteristicsRelevant;
        break;
    case PicamParameter_CleanBeforeExposure:
        driverParameter = PICAM_CleanBeforeExposureRelevant;
        break;
    case PicamParameter_CleanCycleCount:
        driverParameter = PICAM_CleanCycleCountRelevant;
        break;
    case PicamParameter_CleanCycleHeight:
        driverParameter = PICAM_CleanCycleHeightRelevant;
        break;
    case PicamParameter_CleanSectionFinalHeight:
        driverParameter = PICAM_CleanSectionFinalHeightRelevant;
        break;
    case PicamParameter_CleanSectionFinalHeightCount:
        driverParameter = PICAM_CleanSectionFinalHeightCountRelevant;
        break;
    case PicamParameter_CleanSerialRegister:
        driverParameter = PICAM_CleanSerialRegisterRelevant;
        break;
    case PicamParameter_CleanUntilTrigger:
        driverParameter = PICAM_CleanUntilTriggerRelevant;
        break;
    case PicamParameter_CorrectPixelBias:
        driverParameter = PICAM_CorrectPixelBiasRelevant;
        break;
    case PicamParameter_CustomModulationSequence:
        driverParameter = PICAM_CustomModulationSequenceRelevant;
        break;
    case PicamParameter_DifEndingGate:
        driverParameter = PICAM_DifEndingGateRelevant;
        break;
    case PicamParameter_DifStartingGate:
        driverParameter = PICAM_DifStartingGateRelevant;
        break;
    case PicamParameter_DisableCoolingFan:
        driverParameter = PICAM_DisableCoolingFanRelevant;
        break;
    case PicamParameter_DisableDataFormatting:
        driverParameter = PICAM_DisableDataFormattingRelevant;
        break;
    case PicamParameter_EMIccdGain:
        driverParameter = PICAM_EMIccdGainRelevant;
        break;
    case PicamParameter_EMIccdGainControlMode:
        driverParameter = PICAM_EMIccdGainControlModeRelevant;
        break;
    case PicamParameter_EnableIntensifier:
        driverParameter = PICAM_EnableIntensifierRelevant;
        break;
    case PicamParameter_EnableModulation:
        driverParameter = PICAM_EnableModulationRelevant;
        break;
    case PicamParameter_EnableModulationOutputSignal:
        driverParameter = PICAM_EnableModulationOutputSignalRelevant;
        break;
    case PicamParameter_EnableNondestructiveReadout:
        driverParameter = PICAM_EnableNondestructiveReadoutRelevant;
        break;
    case PicamParameter_EnableSensorWindowHeater:
        driverParameter = PICAM_EnableSensorWindowHeaterRelevant;
        break;
    case PicamParameter_EnableSyncMaster:
        driverParameter = PICAM_EnableSyncMasterRelevant;
        break;
    case PicamParameter_ExactReadoutCountMaximum:
        driverParameter = PICAM_ExactReadoutCountMaximumRelevant;
        break;
    case PicamParameter_ExposureTime:
        driverParameter = PICAM_ExposureTimeRelevant;
        break;
    case PicamParameter_FrameRateCalculation:
        driverParameter = PICAM_FrameRateCalculationRelevant;
        break;
    case PicamParameter_FrameSize:
        driverParameter = PICAM_FrameSizeRelevant;
        break;
    case PicamParameter_FramesPerReadout:
        driverParameter = PICAM_FramesPerReadoutRelevant;
        break;
    case PicamParameter_FrameStride:
        driverParameter = PICAM_FrameStrideRelevant;
        break;
    case PicamParameter_FrameTrackingBitDepth:
        driverParameter = PICAM_FrameTrackingBitDepthRelevant;
        break;
    case PicamParameter_GateTracking:
        driverParameter = PICAM_GateTrackingRelevant;
        break;
    case PicamParameter_GateTrackingBitDepth:
        driverParameter = PICAM_GateTrackingBitDepthRelevant;
        break;
    case PicamParameter_GatingMode:
        driverParameter = PICAM_GatingModeRelevant;
        break;
    case PicamParameter_GatingSpeed:
        driverParameter = PICAM_GatingSpeedRelevant;
        break;
    case PicamParameter_IntensifierDiameter:
        driverParameter = PICAM_IntensifierDiameterRelevant;
        break;
    case PicamParameter_IntensifierGain:
        driverParameter = PICAM_IntensifierGainRelevant;
        break;
    case PicamParameter_IntensifierOptions:
        driverParameter = PICAM_IntensifierOptionsRelevant;
        break;
    case PicamParameter_IntensifierStatus:
        driverParameter = PICAM_IntensifierStatusRelevant;
        break;
    case PicamParameter_InvertOutputSignal:
        driverParameter = PICAM_InvertOutputSignalRelevant;
        break;
    case PicamParameter_KineticsWindowHeight:
        driverParameter = PICAM_KineticsWindowHeightRelevant;
        break;
    case PicamParameter_MaskedBottomMargin:
        driverParameter = PICAM_MaskedBottomMarginRelevant;
        break;
    case PicamParameter_MaskedHeight:
        driverParameter = PICAM_MaskedHeightRelevant;
        break;
    case PicamParameter_MaskedTopMargin:
        driverParameter = PICAM_MaskedTopMarginRelevant;
        break;
    case PicamParameter_ModulationDuration:
        driverParameter = PICAM_ModulationDurationRelevant;
        break;
    case PicamParameter_ModulationFrequency:
        driverParameter = PICAM_ModulationFrequencyRelevant;
        break;
    case PicamParameter_ModulationOutputSignalAmplitude:
        driverParameter = PICAM_EnableModulationOutputSignalAmplitudeRelevant;
        break;
    case PicamParameter_ModulationOutputSignalFrequency:
        driverParameter = PICAM_EnableModulationOutputSignalFrequencyRelevant;
        break;
    case PicamParameter_ModulationTracking:
        driverParameter = PICAM_ModulationTrackingRelevant;
        break;
    case PicamParameter_ModulationTrackingBitDepth:
        driverParameter = PICAM_ModulationTrackingBitDepthRelevant;
        break;
    case PicamParameter_NondestructiveReadoutPeriod:
        driverParameter = PICAM_NondestructiveReadoutPeriodRelevant;
        break;
    case PicamParameter_NormalizeOrientation:
        driverParameter = PICAM_NormalizeOrientationRelevant;
        break;
    case PicamParameter_OnlineReadoutRateCalculation:
        driverParameter = PICAM_OnlineReadoutRateCalculationRelevant;
        break;
    case PicamParameter_Orientation:
        driverParameter = PICAM_OrientationRelevant;
        break;
    case PicamParameter_OutputSignal:
        driverParameter = PICAM_OutputSignalRelevant;
        break;
    case PicamParameter_PhosphorDecayDelay:
        driverParameter = PICAM_PhosphorDecayDelayRelevant;
        break;
    case PicamParameter_PhosphorDecayDelayResolution:
        driverParameter = PICAM_PhosphorDecayDelayResolutionRelevant;
        break;
    case PicamParameter_PhosphorType:
        driverParameter = PICAM_PhosphorTypeRelevant;
        break;
    case PicamParameter_PhotocathodeSensitivity:
        driverParameter = PICAM_PhotocathodeSensitivityRelevant;
        break;
    case PicamParameter_PhotonDetectionMode:
        driverParameter = PICAM_PhotonDetectionModeRelevant;
        break;
    case PicamParameter_PhotonDetectionThreshold:
        driverParameter = PICAM_PhotonDetectionThresholdRelevant;
        break;
    case PicamParameter_PixelBitDepth:
        driverParameter = PICAM_PixelBitDepthRelevant;
        break;
    case PicamParameter_PixelFormat:
        driverParameter = PICAM_PixelFormatRelevant;
        break;
    case PicamParameter_PixelGapHeight:
        driverParameter = PICAM_PixelGapHeightRelevant;
        break;
    case PicamParameter_PixelGapWidth:
        driverParameter = PICAM_PixelGapWidthRelevant;
        break;
    case PicamParameter_PixelHeight:
        driverParameter = PICAM_PixelHeightRelevant;
        break;
    case PicamParameter_PixelWidth:
        driverParameter = PICAM_PixelWidthRelevant;
        break;
    case PicamParameter_ReadoutControlMode:
        driverParameter = PICAM_ReadoutControlModeRelevant;
        break;
    case PicamParameter_ReadoutCount:
        driverParameter = PICAM_ReadoutCountRelevant;
        break;
    case PicamParameter_ReadoutOrientation:
        driverParameter = PICAM_ReadoutOrientationRelevant;
        break;
    case PicamParameter_ReadoutPortCount:
        driverParameter = PICAM_ReadoutPortCountRelevant;
        break;
    case PicamParameter_ReadoutRateCalculation:
        driverParameter = PICAM_ReadoutRateCalculationRelevant;
        break;
    case PicamParameter_ReadoutStride:
        driverParameter = PICAM_ReadoutStrideRelevant;
        break;
    case PicamParameter_ReadoutTimeCalculation:
        driverParameter = PICAM_ReadoutTimeCalculationRelevant;
        break;
    case PicamParameter_RepetitiveGate:
        driverParameter = PICAM_RepetitiveGateRelevant;
        break;
    case PicamParameter_RepetitiveModulationPhase:
        driverParameter = PICAM_RepetitiveModulationPhaseRelevant;
        break;
    case PicamParameter_Rois:
        driverParameter = PICAM_RoisRelevant;
        break;
    case PicamParameter_SecondaryActiveHeight:
        driverParameter = PICAM_SecondaryActiveHeightRelevant;
        break;
    case PicamParameter_SecondaryMaskedHeight:
        driverParameter = PICAM_SecondaryMaskedHeightRelevant;
        break;
    case PicamParameter_SensorActiveBottomMargin:
        driverParameter = PICAM_SensorActiveBottomMarginRelevant;
        break;
    case PicamParameter_SensorActiveHeight:
        driverParameter = PICAM_SensorActiveHeightRelevant;
        break;
    case PicamParameter_SensorActiveLeftMargin:
        driverParameter = PICAM_SensorActiveLeftMarginRelevant;
        break;
    case PicamParameter_SensorActiveRightMargin:
        driverParameter = PICAM_SensorActiveRightMarginRelevant;
        break;
    case PicamParameter_SensorActiveTopMargin:
        driverParameter = PICAM_SensorActiveTopMarginRelevant;
        break;
    case PicamParameter_SensorActiveWidth:
        driverParameter = PICAM_SensorActiveWidthRelevant;
        break;
    case PicamParameter_SensorMaskedBottomMargin:
        driverParameter = PICAM_SensorMaskedBottomMarginRelevant;
        break;
    case PicamParameter_SensorMaskedHeight:
        driverParameter = PICAM_SensorMaskedHeightRelevant;
        break;
    case PicamParameter_SensorMaskedTopMargin:
        driverParameter = PICAM_SensorMaskedTopMarginRelevant;
        break;
    case PicamParameter_SensorSecondaryActiveHeight:
        driverParameter = PICAM_SensorSecondaryActiveHeightRelevant;
        break;
    case PicamParameter_SensorSecondaryMaskedHeight:
        driverParameter = PICAM_SensorSecondaryMaskedHeightRelevant;
        break;
    case PicamParameter_SensorTemperatureReading:
        driverParameter = PICAM_SensorTemperatureReadingRelevant;
        break;
    case PicamParameter_SensorTemperatureSetPoint:
        driverParameter = PICAM_SensorTemperatureSetPointRelevant;
        break;
    case PicamParameter_SensorTemperatureStatus:
        driverParameter = PICAM_SensorTemperatureStatusRelevant;
        break;
    case PicamParameter_SensorType:
        driverParameter = PICAM_SensorTypeRelevant;
        break;
    case PicamParameter_SequentialEndingGate:
        driverParameter = PICAM_SequentialEndingGateRelevant;
        break;
    case PicamParameter_SequentialEndingModulationPhase:
        driverParameter = PICAM_SequentialEndingModulationPhaseRelevant;
        break;
    case PicamParameter_SequentialGateStepCount:
        driverParameter = PICAM_SequentialGateStepCountRelevant;
        break;
    case PicamParameter_SequentialGateStepIterations:
        driverParameter = PICAM_SequentialGateStepIterationsRelevant;
        break;
    case PicamParameter_SequentialStartingGate:
        driverParameter = PICAM_SequentialStartingGateRelevant;
        break;
    case PicamParameter_SequentialStartingModulationPhase:
        driverParameter = PICAM_SequentialStartingModulationPhaseRelevant;
        break;
    case PicamParameter_ShutterClosingDelay:
        driverParameter = PICAM_ShutterClosingDelayRelevant;
        break;
    case PicamParameter_ShutterDelayResolution:
        driverParameter = PICAM_ShutterDelayResolutionRelevant;
        break;
    case PicamParameter_ShutterOpeningDelay:
        driverParameter = PICAM_ShutterOpeningDelayRelevant;
        break;
    case PicamParameter_ShutterTimingMode:
        driverParameter = PICAM_ShutterTimingModeRelevant;
        break;
    case PicamParameter_SyncMaster2Delay:
        driverParameter = PICAM_SyncMaster2DelayRelevant;
        break;
    case PicamParameter_TimeStampBitDepth:
        driverParameter = PICAM_TimeStampBitDepthRelevant;
        break;
    case PicamParameter_TimeStampResolution:
        driverParameter = PICAM_TimeStampResolutionRelevant;
        break;
    case PicamParameter_TimeStamps:
        driverParameter = PICAM_TimeStampsRelevant;
        break;
    case PicamParameter_TrackFrames:
        driverParameter = PICAM_TrackFramesRelevant;
        break;
    case PicamParameter_TriggerCoupling:
        driverParameter = PICAM_TriggerCouplingRelevant;
        break;
    case PicamParameter_TriggerDetermination:
        driverParameter = PICAM_TriggerDeterminationRelevant;
        break;
    case PicamParameter_TriggerFrequency:
        driverParameter = PICAM_TriggerFrequencyRelevant;
        break;
    case PicamParameter_TriggerResponse:
        driverParameter = PICAM_TriggerResponseRelevant;
        break;
    case PicamParameter_TriggerSource:
        driverParameter = PICAM_TriggerSourceRelevant;
        break;
    case PicamParameter_TriggerTermination:
        driverParameter = PICAM_TriggerTerminationRelevant;
        break;
    case PicamParameter_TriggerThreshold:
        driverParameter = PICAM_TriggerThresholdRelevant;
        break;
    case PicamParameter_VerticalShiftRate:
        driverParameter = PICAM_VerticalShiftRateRelevant;
        break;
    default:
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter, parameter,
                &string);
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "---- Can't find parameter %s\n",
                string);
        Picam_DestroyString(string);
        break;
    }
    setIntegerParam(driverParameter, relevence);
    return (asynStatus) status;
}

/**
 * Change the parameter values based on those stored in the camera as the
 * selected detector changes.
 */
asynStatus ADPICam::piSetParameterValuesFromSelectedCamera() {
    int status = asynSuccess;
    PicamError error;
    const pichar *paramString;
    piint parameterCount;
    const PicamParameter *parameterList;
    static const char *functionName = "piSetParameterValuesFromSelectedCamera";
    int driverParam = -1;
    PicamValueType paramType;

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s Enter\n", driverName,
            functionName);

    Picam_GetParameters(currentCameraHandle, &parameterList, &parameterCount);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s %d parameters found\n",
            driverName, functionName, parameterCount);

    for (int ii = 0; ii < parameterCount; ii++) {
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
                parameterList[ii], &paramString);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "---- Found %s\n",
                paramString);
        Picam_DestroyString(paramString);
        driverParam = piLookupDriverParameter(parameterList[ii]);
        pibln doesParamExist;
        Picam_DoesParameterExist(currentCameraHandle,
        		parameterList[ii],
				&doesParamExist);
        if (doesParamExist) {
			if (driverParam >= 0) {
				error = Picam_GetParameterValueType(currentCameraHandle,
						parameterList[ii], &paramType);
				if (error != PicamError_None) {
					//TODO
				}
				PicamConstraintType constraintType;
				Picam_GetParameterConstraintType(currentCameraHandle,
						parameterList[ii],
						&constraintType);
				if (constraintType == PicamConstraintType_Collection){
					const pichar *paramString;
					Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
							parameterList[ii],
							&paramString);
					asynPrint (pasynUserSelf, ASYN_TRACE_FLOW,
							"%s:%s Updating list for %s\n",
							driverName,
							functionName,
							paramString);
					Picam_DestroyString(paramString);
					piUpdateParameterListValues(parameterList[ii], driverParam);
				}
				switch (paramType) {
				case PicamValueType_Integer:
					piint intVal;
					error = Picam_GetParameterIntegerValue(currentCameraHandle,
							parameterList[ii], &intVal);
					setIntegerParam(driverParam, intVal);
					break;
				case PicamValueType_Enumeration:
					if (constraintType != PicamConstraintType_None) {
						error = Picam_GetParameterIntegerValue(currentCameraHandle,
								parameterList[ii], &intVal);
						setIntegerParam(driverParam, intVal);
					}
					else {
						PicamEnumeratedType picamET;
						const char *enumString;
						int intValue;
						Picam_GetParameterEnumeratedType(currentCameraHandle,
								parameterList[ii],
								&picamET);
						Picam_GetParameterIntegerValue(currentCameraHandle,
												parameterList[ii], &intValue);
						Picam_GetEnumerationString(picamET, intValue, &enumString);
						setStringParam(driverParam, enumString);
						Picam_DestroyString(enumString);
					}
					break;
				case PicamValueType_LargeInteger:
					pi64s largeVal;
					int val;
					error = Picam_GetParameterLargeIntegerValue(currentCameraHandle,
							parameterList[ii], &largeVal);
					val = (int)largeVal;
					setIntegerParam(driverParam, val);
					break;
				case PicamValueType_FloatingPoint:
					piflt fltVal;
					const PicamCollectionConstraint *speedConstraint;
					if (parameterList[ii] == PicamParameter_AdcSpeed) {
						error = Picam_GetParameterCollectionConstraint(
								currentCameraHandle, PicamParameter_AdcSpeed,
								PicamConstraintCategory_Capable, &speedConstraint);
						error = Picam_GetParameterFloatingPointValue(
								currentCameraHandle, PicamParameter_AdcSpeed,
								&fltVal);
						for (int ii = 0; ii < speedConstraint->values_count; ii++) {
							if (speedConstraint->values_array[ii] == fltVal) {
								setIntegerParam(driverParam, ii);
							}
						}
					} else {
						error = Picam_GetParameterFloatingPointValue(
								currentCameraHandle, parameterList[ii], &fltVal);
						setDoubleParam(driverParam, fltVal);

					}
					break;
				}

			} else if (parameterList[ii] == PicamParameter_Rois) {
				const PicamRois *paramRois;
				const PicamRoisConstraint *roiConstraint;
				error = Picam_GetParameterRoisValue(currentCameraHandle,
						parameterList[ii], &paramRois);
				error = Picam_GetParameterRoisConstraint(currentCameraHandle,
						parameterList[ii], PicamConstraintCategory_Capable,
						&roiConstraint);
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Rois %d, rules 0X%X\n",
						paramRois->roi_count,  roiConstraint->rules);

				if (paramRois->roi_count == 1) {
					setIntegerParam(ADBinX, paramRois->roi_array[0].x_binning);
					setIntegerParam(ADBinY, paramRois->roi_array[0].y_binning);
					setIntegerParam(ADMinX, paramRois->roi_array[0].x);
					setIntegerParam(ADMinY, paramRois->roi_array[0].y);
					setIntegerParam(ADSizeX, paramRois->roi_array[0].width);
					setIntegerParam(ADSizeY, paramRois->roi_array[0].height);
					setIntegerParam(NDArraySizeX, paramRois->roi_array[0].width);
					setIntegerParam(NDArraySizeY, paramRois->roi_array[0].height);
					if (roiConstraint->rules & PicamRoisConstraintRulesMask_HorizontalSymmetry) {
						setIntegerParam(PICAM_EnableROIMinXInput, 0);
					}
					else {
						setIntegerParam(PICAM_EnableROIMinXInput, 1);
					}
					if (roiConstraint->rules & PicamRoisConstraintRulesMask_VerticalSymmetry) {
						setIntegerParam(PICAM_EnableROIMinYInput, 0);
					}
					else {
						setIntegerParam(PICAM_EnableROIMinYInput, 1);
					}
				}
			}
        }
    }

    Picam_DestroyParameters(parameterList);
    callParamCallbacks();

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s Exit\n", driverName,
            functionName);

    return (asynStatus) status;
}

/**
 Set values associated with selected detector
 */
asynStatus ADPICam::piSetRois(int minX, int minY, int width, int height,
        int binX, int binY) {
    int status = asynSuccess;
    const PicamRois *rois;
    const PicamRoisConstraint *roisConstraints;
    const char *functionName = "piSetRois";
    int numXPixels, numYPixels;
    const pichar *errorString;

    PicamError error = PicamError_None;
    getIntegerParam(ADMaxSizeX, & numXPixels);
    getIntegerParam(ADMaxSizeY, & numYPixels);
    error = Picam_GetParameterRoisValue(currentCameraHandle,
            PicamParameter_Rois, &rois);
    if (error != PicamError_None) {
        Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                &errorString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Error retrieving rois %s\n", driverName, functionName,
                errorString);
        Picam_DestroyString(errorString);
        return asynError;
    }
    error = Picam_GetParameterRoisConstraint(currentCameraHandle,
            PicamParameter_Rois, PicamConstraintCategory_Required,
            &roisConstraints);
    printf ("ROIConstraints->rules 0x%X\n", roisConstraints->rules);
    if (error != PicamError_None) {
        Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                &errorString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Error retrieving rois constraints %s\n", driverName,
                functionName, errorString);
        Picam_DestroyString(errorString);
        return asynError;
    }
    if (rois->roi_count == 1) {
        PicamRoi *roi = &(rois->roi_array[0]);
        bool allInRange = true;
        if (binX == 0) binX = 1;
        if (roisConstraints->rules & PicamRoisConstraintRulesMask_HorizontalSymmetry) {
        	if (width >= numXPixels/binX){
        		width = numXPixels/binX;
        	}
        	//make sure pixels in each quadrant are divisible by binnning
        	if (((width/2)/binX) * binX != width){
    			width = (((width/2)/binX) * binX)*2;
    		}
        	roi->x = ((numXPixels + 1)/2 ) - ((width/2) * binX) ;
        	roi->width = width * binX;
        	roi->x_binning = binX;
        	setIntegerParam(ADMinX, roi->x);
        	setIntegerParam(ADSizeX, width);
        	setIntegerParam(ADBinX, binX);
        }
        else {

        	if (minX < 1) {
                minX = 1;
            } else if (minX > numXPixels-binX) {
            	minX = numXPixels;
            }
            roi->x = minX;
            setIntegerParam(ADMinX, minX);
            if (width < 1){
                width = 1;
            } else if (width > (numXPixels - minX + 1)/binX) {
            	width = (numXPixels - minX + 1)/binX;
            	if (width < 1) {
            		width = 1;
            	}
            }
            roi->width = width * binX;
            roi->x_binning = binX;
            setIntegerParam(ADSizeX, width);
            setIntegerParam(ADBinX, binX);

        }
        if (binY == 0) binY = 1;
        if (roisConstraints->rules & PicamRoisConstraintRulesMask_VerticalSymmetry) {
        	if (height >= numYPixels/binY ){
        		height = numYPixels/binY;
        	}
        	//make sure pixels in each quadrant are divisible by binnning
        	if (((height/2)/binY) * binY != height){
    			height = (((height/2)/binY) * binY)*2;
    		}
        	roi->y = ((numYPixels + 1)/2 ) - ((height/2) * binY) ;
        	roi->height = height * binY;
        	roi->y_binning = binY;
        	setIntegerParam(ADMinY, roi->y);
        	setIntegerParam(ADSizeY, height);
        	setIntegerParam(ADBinY, binY);
        }
        else {

        	if (minY < 1){
				minY = 1;
			}
			else if (minY > numYPixels){
				minY = numYPixels;
			}
			roi->y = minY;
			setIntegerParam(ADMinY, minY);
			if (height > (numYPixels - minY + 1)/binY) {
				height = (numYPixels - minY + 1)/binY;
				if (height < 1) {
					height = 1;
				}
			} else if (height < 1 ) {
				height = 1;
			}
			roi->height = height * binY;
			roi->y_binning = binY;
			setIntegerParam(ADSizeY, height);
			setIntegerParam(ADBinY, binY);
        }
        error = Picam_SetParameterRoisValue(currentCameraHandle,
                PicamParameter_Rois, rois);
        if (error != PicamError_None) {
            Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                    &errorString);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s Error writing rois %s\n"
            		"(x,y) = (%d, %d), (height, width) = (%d, %d), "
            		"(xbin, ybin) = (%d, %d)\n"
            		, driverName,
					functionName,
                    errorString,
					roi->x,
					roi->y,
					roi->width,
					roi->height,
					roi->x_binning,
					roi->y_binning);

            Picam_DestroyString(errorString);
            return asynError;
        }
    }
    callParamCallbacks();
    Picam_DestroyRoisConstraints(roisConstraints);
    Picam_DestroyRois(rois);
    return (asynStatus) status;
}

/**
 * Set the selected camera based on user input
 */
asynStatus ADPICam::piSetSelectedCamera(asynUser *pasynUser,
        int selectedIndex) {
    int status = asynSuccess;
    const PicamFirmwareDetail *firmwareDetails;
    PicamError error;
    piint numFirmwareDetails = 0;
    const char *modelString;
    const char *interfaceString;
    static const char *functionName = "piSetSelectedCamera";
    char enumString[64];
    char firmwareString[64];

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: Selected camera value=%d\n",
            driverName, functionName, selectedIndex);
    if (currentCameraHandle != NULL) {
        piUnregisterRelevantWatch(currentCameraHandle);
        piUnregisterConstraintChangeWatch(currentCameraHandle);
        piUnregisterValueChangeWatch(currentCameraHandle);
        error = PicamAdvanced_UnregisterForAcquisitionUpdated(
        		currentDeviceHandle,
                piAcquistionUpdated);
        if (error != PicamError_None){
            const char *errorString;
            Picam_GetEnumerationString(PicamEnumeratedType_Error,
                    error,
                    &errorString);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s Error With Picam_UnregisterForAcquisitionUpdate "
            		"%d: %s\n",
                    driverName,
                    functionName,
                    currentCameraHandle,
                    errorString);
            Picam_DestroyString(errorString);
        }
    }
    if (selectedCameraIndex >= 0) {
        error = Picam_CloseCamera(currentCameraHandle);
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "Picam_CloseCameraError %d\n",
                error);
    }
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s: Number of available cameras=%d\n", driverName, functionName,
            availableCamerasCount);
    for (int ii = 0; ii < availableCamerasCount; ii++) {
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "Available Camera[%d]\n", ii);
        Picam_GetEnumerationString(PicamEnumeratedType_Model,
                (piint) availableCameraIDs[ii].model, &modelString);
        sprintf(enumString, "%s", modelString);
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "\n---%s\n---%d\n---%s\n---%s\n",
                modelString, availableCameraIDs[ii].computer_interface,
                availableCameraIDs[ii].sensor_name,
                availableCameraIDs[ii].serial_number);
        Picam_DestroyString(modelString);
        //PicamAdvanced_SetUserState(availableCameraIDs[ii].model, this);
    }
    if (selectedIndex < availableCamerasCount) {

        selectedCameraIndex = selectedIndex;
        error = Picam_OpenCamera(&(availableCameraIDs[selectedIndex]),
                &currentCameraHandle);

        if (error != PicamError_None) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s Trouble Opening Camera\n", driverName, functionName);
        }
        error = PicamAdvanced_GetCameraDevice(currentCameraHandle,
                &currentDeviceHandle);
        if (error != PicamError_None) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s Trouble Getting Camera Device\n",
                    driverName,
                    functionName);
        }

        error = PicamAdvanced_RegisterForAcquisitionUpdated(currentDeviceHandle,
                piAcquistionUpdated);
        if (error != PicamError_None){
            const char *errorString;
            Picam_GetEnumerationString(PicamEnumeratedType_Error,
                    error,
                    &errorString);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s Error With Picam_RegisterForAcquisitionUpdate: %s\n",
                    driverName,
                    functionName,
                    errorString);
            Picam_DestroyString(errorString);
        }


        Picam_GetEnumerationString(PicamEnumeratedType_Model,
                (piint) availableCameraIDs[selectedIndex].model, &modelString);
        sprintf(enumString, "%s", modelString);
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                "%s:%s: Selected camera value=%d, %s\n", driverName,
                functionName, selectedIndex, modelString);
        Picam_DestroyString(modelString);

        Picam_GetEnumerationString(PicamEnumeratedType_ComputerInterface,
                (piint) availableCameraIDs[selectedIndex].computer_interface,
                &interfaceString);
        sprintf(enumString, "%s", interfaceString);
        status |= setStringParam(PICAM_CameraInterface, enumString);
        Picam_DestroyString(interfaceString);

        status |= setIntegerParam(PICAM_AvailableCameras, selectedIndex);
        status |= setStringParam(PICAM_SensorName,
                availableCameraIDs[selectedIndex].sensor_name);
        status |= setStringParam(PICAM_SerialNumber,
                availableCameraIDs[selectedIndex].serial_number);

        Picam_GetFirmwareDetails(&(availableCameraIDs[selectedIndex]),
                &firmwareDetails, &numFirmwareDetails);
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "----%d\n", numFirmwareDetails);
        if (numFirmwareDetails > 0) {
            asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s\n",
                    firmwareDetails[0].detail);

            sprintf(firmwareString, "%s", firmwareDetails[0].detail);

            Picam_DestroyFirmwareDetails(firmwareDetails);
            status |= setStringParam(PICAM_FirmwareRevision, firmwareString);
        } else {
            status |= setStringParam(PICAM_FirmwareRevision, "N/A");
        }
    } else {
        setIntegerParam(PICAM_AvailableCameras, 0);
        setIntegerParam(PICAM_UnavailableCameras, 0);

    }
    callParamCallbacks();

    //piSetParameterValuesFromSelectedCamera();

    piRegisterValueChangeWatch(currentCameraHandle);

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    status |= piClearParameterExists();
    status |= piClearParameterRelevance();
    status |= piUpdateParameterExists();
    status |= piUpdateParameterRelevance();
    piRegisterRelevantWatch(currentCameraHandle);

    return (asynStatus) status;
}

/**
 * set the selected unavailable camera (to show camera info) based on user
 * input
 */
asynStatus ADPICam::piSetSelectedUnavailableCamera(asynUser *pasynUser,
        int selectedIndex) {
    int status = asynSuccess;
    const PicamFirmwareDetail *firmwareDetails;
    piint numFirmwareDetails = 0;
    const char *modelString;
    const char *interfaceString;
    static const char *functionName = "piSetSelectedUnavailableCamera";
    char enumString[64];
    char firmwareString[64];

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: Entry\n", driverName,
            functionName);
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: Selected camera value=%d\n",
            driverName, functionName, selectedIndex);
    if (unavailableCamerasCount == 0) {
        asynPrint(pasynUser, ASYN_TRACE_WARNING,
                "%s:%s: There are no unavailable cameras\n", driverName,
                functionName);
        status |= setStringParam(PICAM_CameraInterfaceUnavailable, enumString);
        status |= setStringParam(PICAM_SensorNameUnavailable, notAvailable);
        status |= setStringParam(PICAM_SerialNumberUnavailable, notAvailable);
        status |= setStringParam(PICAM_FirmwareRevisionUnavailable,
                notAvailable);
        return asynSuccess;
    }
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s:%s: Number of available cameras=%d\n", driverName, functionName,
            unavailableCamerasCount);

    Picam_GetEnumerationString(PicamEnumeratedType_Model,
            (piint) unavailableCameraIDs[selectedIndex].model, &modelString);
    sprintf(enumString, "%s", modelString);
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s: Selected camera value=%d, %s\n", driverName, functionName,
            selectedIndex, modelString);
    Picam_DestroyString(modelString);

    Picam_GetEnumerationString(PicamEnumeratedType_ComputerInterface,
            (piint) unavailableCameraIDs[selectedIndex].computer_interface,
            &interfaceString);
    sprintf(enumString, "%s", interfaceString);
    status |= setStringParam(PICAM_CameraInterfaceUnavailable, enumString);
    Picam_DestroyString(interfaceString);

    status |= setIntegerParam(PICAM_AvailableCameras, selectedIndex);
    status |= setStringParam(PICAM_SensorNameUnavailable,
            unavailableCameraIDs[selectedIndex].sensor_name);
    status |= setStringParam(PICAM_SerialNumberUnavailable,
            unavailableCameraIDs[selectedIndex].serial_number);

    Picam_GetFirmwareDetails(&(unavailableCameraIDs[selectedIndex]),
            &firmwareDetails, &numFirmwareDetails);
    asynPrint(pasynUser, ASYN_TRACE_FLOW, "----%d\n", numFirmwareDetails);
    if (numFirmwareDetails > 0) {
        asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s\n",
                firmwareDetails[0].detail);

        sprintf(firmwareString, "%s", firmwareDetails[0].detail);

        Picam_DestroyFirmwareDetails(firmwareDetails);
        status |= setStringParam(PICAM_FirmwareRevisionUnavailable,
                firmwareString);
    } else {
        status |= setStringParam(PICAM_FirmwareRevisionUnavailable, "N/A");
    }

    asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: Entry\n", driverName,
            functionName);
    return (asynStatus) status;
}

/**
 * Unregister constraint change callbacks for the currently selected detector
 *
 */
asynStatus ADPICam::piUnregisterConstraintChangeWatch(
        PicamHandle cameraHandle) {
    int status = asynSuccess;
    piint parameterCount;
    const PicamParameter *parameterList;
    PicamError error;

    error = Picam_GetParameters(currentCameraHandle, &parameterList,
            &parameterCount);
    if (error != PicamError_None) {
        //TODO
    }
    for (int ii = 0; ii < parameterCount; ii++) {
        error = PicamAdvanced_UnregisterForIsRelevantChanged(cameraHandle,
                parameterList[ii], piParameterRelevanceChanged);
    }
    return (asynStatus) status;
}

/**
 * Unregister Parameter Relevance callback for parameters in the currently
 * selected camera
 */
asynStatus ADPICam::piUnregisterRelevantWatch(PicamHandle cameraHandle) {
    int status = asynSuccess;
    piint parameterCount;
    const PicamParameter *parameterList;
    PicamError error;

    error = Picam_GetParameters(currentCameraHandle, &parameterList,
            &parameterCount);
    if (error != PicamError_None) {
        //TODO
    }
    for (int ii = 0; ii < parameterCount; ii++) {
        error = PicamAdvanced_UnregisterForIsRelevantChanged(cameraHandle,
                parameterList[ii], piParameterRelevanceChanged);
    }
    return (asynStatus) status;
}

/**
 * Unregister Parameter Value Change callbacks for the currently selected
 * camera
 */
asynStatus ADPICam::piUnregisterValueChangeWatch(PicamHandle cameraHandle) {
    int status = asynSuccess;
    piint parameterCount;
    const PicamParameter *parameterList;
    PicamValueType valueType;
    PicamError error;
    const char *functionName = "piUnregisterValueChangeWatch";
    pibln doesParamExist;

    error = Picam_GetParameters(currentCameraHandle, &parameterList,
            &parameterCount);
    if (error != PicamError_None) {
        //TODO
    }
    for (int ii = 0; ii < parameterCount; ii++) {
    	Picam_DoesParameterExist(currentCameraHandle,
    			parameterList[ii],
				&doesParamExist);
    	if (doesParamExist){
			error = Picam_GetParameterValueType(cameraHandle, parameterList[ii],
					&valueType);
			if (error != PicamError_None) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s \n", driverName,
						functionName);
				return asynError;
			}
			switch (valueType) {
			case PicamValueType_Integer:
			case PicamValueType_Boolean:
			case PicamValueType_Enumeration:
				error = PicamAdvanced_UnregisterForIntegerValueChanged(cameraHandle,
						parameterList[ii], piParameterIntegerValueChanged);
				break;
			case PicamValueType_LargeInteger:
				error = PicamAdvanced_UnregisterForLargeIntegerValueChanged(
						cameraHandle, parameterList[ii],
						piParameterLargeIntegerValueChanged);
				break;
			case PicamValueType_FloatingPoint:
				error = PicamAdvanced_UnregisterForFloatingPointValueChanged(
						cameraHandle, parameterList[ii],
						piParameterFloatingPointValueChanged);
				break;
			case PicamValueType_Rois:
				printf("Unregistering ROIS value change\n");
				error = PicamAdvanced_UnregisterForRoisValueChanged(cameraHandle,
						parameterList[ii], piParameterRoisValueChanged);
				break;
			case PicamValueType_Pulse:
				error = PicamAdvanced_UnregisterForPulseValueChanged(cameraHandle,
						parameterList[ii], piParameterPulseValueChanged);
				break;
			case PicamValueType_Modulations:
				error = PicamAdvanced_UnregisterForModulationsValueChanged(
						cameraHandle, parameterList[ii],
						piParameterModulationsValueChanged);
				break;
			default: {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
						"%s:%s Unexpected valueType %s", driverName, functionName,
						valueType);
				return asynError;
			}
				break;
			}
    	}
    }
    return (asynStatus) status;
}

/**
 Update PICAM parameter existance for the current detector
 */
asynStatus ADPICam::piUpdateParameterExists() {
    int status = asynSuccess;
    piint parameterCount;
    const PicamParameter *parameterList;
    const pichar* string;
    Picam_GetParameters(currentCameraHandle, &parameterList, &parameterCount);
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%d parameters found\n",
            parameterCount);

    for (int ii = 0; ii < parameterCount; ii++) {
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
                parameterList[ii], &string);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "---- Found %s\n", string);
        Picam_DestroyString(string);
        status |= piSetParameterExists(pasynUserSelf, parameterList[ii], 1);
    }
    Picam_DestroyParameters(parameterList);
    callParamCallbacks();
    return (asynStatus) status;
}

/**
 Update PICAM parameter relevance for the current detector
 */
asynStatus ADPICam::piUpdateParameterRelevance() {
    int status = asynSuccess;
    piint parameterCount;
    const PicamParameter *parameterList;
    const pichar* string;
    Picam_GetParameters(currentCameraHandle, &parameterList, &parameterCount);
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%d parameters found\n",
            parameterCount);

    pibln isRelevant;
    int iRelevant;
    for (int ii = 0; ii < parameterCount; ii++) {
        Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
                parameterList[ii], &string);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "---- Found %s\n", string);
        Picam_DestroyString(string);
        Picam_IsParameterRelevant(currentCameraHandle, parameterList[ii],
        		&isRelevant);
        iRelevant = (int)isRelevant;
        status |= piSetParameterRelevance(pasynUserSelf, parameterList[ii],
        		iRelevant);
    }
    Picam_DestroyParameters(parameterList);
    callParamCallbacks();
    return (asynStatus) status;
}

/**
 * Reread the list of available detectors to make the list available to the
 * user as detectors come online/go offline
 */
asynStatus ADPICam::piUpdateAvailableCamerasList() {
    int status = asynSuccess;
    const char *functionName = "piUpdateAvailableCamerasList";
    const char *modelString;
    char enumString[64];
    char *strings[MAX_ENUM_STATES];
    int values[MAX_ENUM_STATES];
    int severities[MAX_ENUM_STATES];
    //size_t nElements;
    size_t nIn;

    nIn = 0;
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s availableCamerasCount %d\n", driverName, functionName,
            availableCamerasCount);
    callParamCallbacks();
    for (int ii = 0; ii < availableCamerasCount; ii++) {
        Picam_GetEnumerationString(PicamEnumeratedType_Model,
                (piint) availableCameraIDs[ii].model, &modelString);
        pibln camConnected = false;
        Picam_IsCameraIDConnected(availableCameraIDs, &camConnected);
        sprintf(enumString, "%s", modelString);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "\n%s:%s: \nCamera[%d]\n---%s\n---%d\n---%s\n---%s\n",
                driverName, functionName, ii, modelString,
                availableCameraIDs[ii].computer_interface,
                availableCameraIDs[ii].sensor_name,
                availableCameraIDs[ii].serial_number);
        Picam_DestroyString(modelString);
        strings[nIn] = epicsStrDup(enumString);
        values[nIn] = ii;
        severities[nIn] = 0;
        (nIn)++;
    }

    doCallbacksEnum(strings, values, severities, nIn, PICAM_AvailableCameras,
            0);
    callParamCallbacks();
    return (asynStatus) status;
}

/**
 *
 */
asynStatus ADPICam::piUpdateParameterListValues(
        PicamParameter picamParameter, int driverParameter){
    const char *functionName = "piUpdateParameterListValues";
    int status = asynSuccess;
    PicamError error;
    const char *errorString;
    char *strings[MAX_ENUM_STATES];
    int values[MAX_ENUM_STATES];
    int severities[MAX_ENUM_STATES];
    size_t nIn;

    const PicamCollectionConstraint *constraints;
    const char *paramConstraintString;

    nIn = 0;

    if (currentCameraHandle != NULL){
    	pibln paramExists;
    	Picam_DoesParameterExist(currentCameraHandle,
    			picamParameter,
				&paramExists);
    	if (paramExists) {
			pibln isRelevant;
			error = Picam_IsParameterRelevant(currentCameraHandle,
					picamParameter,
					&isRelevant);
			if (error == PicamError_ParameterDoesNotExist) {
				// Parameter is not Relevant
				Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
						picamParameter,
						&errorString);
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
						"%s:%s Setting is not Relevant manually function = %d, "
						"picamParam = %d\n",
						driverName,
						functionName,
						driverParameter,
						picamParameter,
						errorString);
				Picam_DestroyString(errorString);
				isRelevant = false;
			}
			else if (error != PicamError_None) {
				Picam_GetEnumerationString(PicamEnumeratedType_Error,
						error,
						&errorString);
				PicamCameraID camID;
				Picam_GetCameraID(currentCameraHandle, &camID);
				const char * cameraModel;
				Picam_GetEnumerationString(PicamEnumeratedType_Model,
						camID.model,
						&cameraModel);
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
						"%s:%s Trouble getting parameter assocoated with driver"
						" param %d, picam param:%d for camera %s: %s\n",
						driverName,
						functionName,
						driverParameter,
						cameraModel,
						errorString);
				Picam_DestroyString(errorString);
				Picam_DestroyString(cameraModel);
				return asynError;
			}
			if (isRelevant) {
				error = Picam_GetParameterCollectionConstraint(
						currentCameraHandle,
						picamParameter,
						PicamConstraintCategory_Capable, &constraints);
				if (error != PicamError_None){
					Picam_GetEnumerationString(PicamEnumeratedType_Error,
							error,
							&errorString);
					asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
							"%s:%s Trouble getting parameter assocoated with "
							"driver param %d, picam param:%d: %s\n",
							driverName,
							functionName,
							driverParameter,
							picamParameter,
							errorString);
					Picam_DestroyString(errorString);
				}

				if ( constraints->values_count == 0){
					strings[0] = epicsStrDup("N.A. Up1");
					values[0] = 0;
					severities[0] = 0;
					(nIn) = 1;
					return asynSuccess;
				}
				for (int ii = 0; ii < constraints->values_count; ii++) {
					PicamEnumeratedType picamParameterET;
					Picam_GetParameterEnumeratedType(currentCameraHandle,
							picamParameter,
							&picamParameterET);
					PicamValueType valType;
					Picam_GetParameterValueType(currentCameraHandle,
							picamParameter, &valType);
					switch (valType)
					{
					case PicamValueType_Enumeration:
						Picam_GetEnumerationString(
							picamParameterET,
							(int)constraints->values_array[ii],
							&paramConstraintString);
						asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
								"%s:%s  -- %s,  driver Parameter: %d\n",
								driverName,
								functionName,
								paramConstraintString,
								driverParameter);
						strings[nIn] = epicsStrDup(paramConstraintString);
						values[nIn] = (int)constraints->values_array[ii];
						Picam_DestroyString(paramConstraintString);
						severities[nIn] = 0;
						(nIn)++;
						break;
					case PicamValueType_FloatingPoint:
						char floatString[12];
						sprintf(floatString, "%f",
								constraints->values_array[ii]);
						strings[nIn] = epicsStrDup(floatString);
						values[nIn] = ii;
						severities[nIn] = 0;
						(nIn)++;
						break;
					case PicamValueType_Integer:
						char intString[12];
						sprintf(intString, "%d",
								(int)constraints->values_array[ii]);
						strings[nIn] = epicsStrDup(intString);
						values[nIn] = (int)constraints->values_array[ii];
						severities[nIn] = 0;
						(nIn)++;
						break;
					case PicamValueType_LargeInteger:
						char largeIntString[12];
						sprintf(largeIntString, "%d",
								(int)constraints->values_array[ii]);
						strings[nIn] = epicsStrDup(largeIntString);
						values[nIn] = (int)constraints->values_array[ii];
						severities[nIn] = 0;
						(nIn)++;
						break;
					case PicamValueType_Boolean:
						strings[nIn] = epicsStrDup(
								constraints->values_array[ii] ? "No":"Yes");
						values[nIn] = constraints->values_array[ii];
						severities[nIn] = 0;
						(nIn)++;
						break;
					default:
						asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
								"%s:%s Fall to defaultType %d %d\n",
								driverName,
								functionName,
								driverParameter,
								picamParameter);
						break;
					}
				}
			}
			else {
				//param Not Relevant
				strings[0] = epicsStrDup("N.A. Up2");
				values[0] = 0;
				severities[0] = 0;
				(nIn) = 1;
			}
    	}
    	else {
    		// param doesn't exist for this detector
			strings[0] = epicsStrDup("N.A. Up2");
			values[0] = 0;
			severities[0] = 0;
			(nIn) = 1;
    	}
    }

    doCallbacksEnum(strings, values, severities, nIn, driverParameter,
            0);

    return (asynStatus)status;
}
/**
 * Update the list of unavailable Camera list.  This is called as detectors go
 * online/offline
 */
asynStatus ADPICam::piUpdateUnavailableCamerasList() {
    int status = asynSuccess;

    return (asynStatus) status;

}

asynStatus ADPICam::piWriteFloat64RangeType(asynUser *pasynUser,
        epicsFloat64 value,
        int driverParameter,
        PicamParameter picamParameter){
    const char *functionName = "piWriteFloat64RangeType";
    int status = asynSuccess;
    PicamError error;
	PicamConstraintType paramCT;
    PicamValueType valType;
	const PicamRangeConstraint *constraint;
    const pichar *errorString;
    const pichar *paramString;

    error = Picam_GetParameterValueType(currentCameraHandle,
    		picamParameter,
			&valType);
    if (error != PicamError_None) {
    	//TODO
    	return asynError;
    }
    if (valType == PicamValueType_FloatingPoint){
    	error = Picam_SetParameterFloatingPointValue(currentCameraHandle,
    			picamParameter,
				value);
    	if (error == PicamError_InvalidParameterValue) {
    		Picam_GetParameterConstraintType( currentCameraHandle,
    				picamParameter,
					&paramCT);
    		if (paramCT == PicamConstraintType_Range) {
    			Picam_GetParameterRangeConstraint(currentCameraHandle,
    					picamParameter,
						PicamConstraintCategory_Required,
						&constraint);
                Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
                        picamParameter,
                        &paramString);
    			if (value < constraint->minimum){
    				asynPrint(pasynUser, ASYN_TRACE_ERROR,
    						"%s,%s Value %f is out of range %f,%f for "
    						"parameter %s\n",
    						driverName,
							functionName,
							value,
							constraint->minimum,
							constraint->maximum,
							paramString);
    				value = (double)constraint->minimum;
    			}
    			if (value > constraint->maximum){
    				asynPrint(pasynUser, ASYN_TRACE_ERROR,
    						"%s,%s Value %f is out of range %f,%f for "
    						"parameter %s\n",
    						driverName,
							functionName,
							value,
							constraint->minimum,
							constraint->maximum,
							paramString);
					value = (double)constraint->maximum;
    			}
    			error = Picam_SetParameterFloatingPointValue(
    					currentCameraHandle,
						picamParameter,
						value);
        		if (error != PicamError_None) {
                    Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                            &errorString);
                    Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
                            picamParameter,
                            &paramString);
                    asynPrint(pasynUser, ASYN_TRACE_ERROR,
                            "%s:%s error writing %d to  %s \n"
                            "Reason %s and not out of range\n",
                            driverName,
                            functionName,
                            value,
                            paramString,
                            errorString);
                    Picam_DestroyString(errorString);
                    return asynError;
                }
                Picam_DestroyString(paramString);

    		}
    	}
    }
    return (asynStatus)status;
}

/**
 * Write int 32 values when a parameters constraint type is range.  This
 * will do some bounds checking according to the range and will not allow
 * setting a value outside of that range.
 *
 */
asynStatus ADPICam::piWriteInt32RangeType(asynUser *pasynUser,
        epicsInt32 value,
        int driverParameter,
        PicamParameter picamParameter){
    const char *functionName = "piWriteInt32RangeType";
    int status = asynSuccess;
    PicamValueType valType;
    PicamError error;
	PicamConstraintType paramCT;
	const PicamRangeConstraint *constraint;
    const pichar *errorString;
    const pichar *paramString;

    error = Picam_GetParameterValueType(currentCameraHandle,
            picamParameter,
            &valType);
    if (error != PicamError_None) {
        // TODO
        return asynError;
    }
    if (valType == PicamValueType_Integer) {
        error = Picam_SetParameterIntegerValue(currentCameraHandle,
                picamParameter, value);
        if (error == PicamError_InvalidParameterValue) {
        	Picam_GetParameterConstraintType(currentCameraHandle,
        			picamParameter,
					&paramCT);
        	if (paramCT == PicamConstraintType_Range){
        		Picam_GetParameterRangeConstraint(currentCameraHandle,
        				picamParameter,
						PicamConstraintCategory_Required,
						&constraint);
                Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
                        picamParameter,
                        &paramString);
        		if (value < constraint->minimum){
    				asynPrint(pasynUser, ASYN_TRACE_ERROR,
    						"%s,%s Value %f is out of range %f,%f for "
    						"parameter %s\n",
    						driverName,
							functionName,
							value,
							constraint->minimum,
							constraint->maximum,
							paramString);
	      			value = (int)constraint->minimum;
        		}
        		else if (value > constraint->maximum){
    				asynPrint(pasynUser, ASYN_TRACE_ERROR,
    						"%s,%s Value %f is out of range %f,%f for "
    						"parameter %s\n",
    						driverName,
							functionName,
							value,
							constraint->minimum,
							constraint->maximum,
							paramString);
	        			value = (int)constraint->maximum;
        		}
                error = Picam_SetParameterIntegerValue(currentCameraHandle,
                        picamParameter, value);
        		if (error != PicamError_None) {
                    Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                            &errorString);
                    asynPrint(pasynUser, ASYN_TRACE_ERROR,
                            "%s:%s error writing %d to  %s \n"
                            "Reason %s and not out of range\n",
                            driverName,
                            functionName,
                            value,
                            paramString,
                            errorString);
                    Picam_DestroyString(errorString);
                    return asynError;
                }
                Picam_DestroyString(paramString);
        	}
        }
        else if (error != PicamError_None) {
            Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                    &errorString);
            Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
                    picamParameter,
                    &paramString);
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s error writing %d to  %s \n"
                    "Reason %s\n",
                    driverName,
                    functionName,
                    value,
                    paramString,
                    errorString);
            Picam_DestroyString(paramString);
            Picam_DestroyString(errorString);
            return asynError;
        }
    }
    if (valType == PicamValueType_LargeInteger) {
        pi64s largeValue;
        largeValue = value;
        error = Picam_SetParameterLargeIntegerValue(currentCameraHandle,
                picamParameter, largeValue);
        if (error == PicamError_InvalidParameterValue) {
        	Picam_GetParameterConstraintType(currentCameraHandle,
        			picamParameter,
					&paramCT);
        	if (paramCT == PicamConstraintType_Range){
        		Picam_GetParameterRangeConstraint(currentCameraHandle,
        				picamParameter,
						PicamConstraintCategory_Required,
						&constraint);
                Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
                        picamParameter,
                        &paramString);
        		if (value < constraint->minimum){
    				asynPrint(pasynUser, ASYN_TRACE_ERROR,
    						"%s,%s Value %f is out of range %f,%f for "
    						"parameter %s\n",
    						driverName,
							functionName,
							value,
							constraint->minimum,
							constraint->maximum,
							paramString);
        			value = (int)constraint->minimum;
        		}
        		else if (value > constraint->maximum){
    				asynPrint(pasynUser, ASYN_TRACE_ERROR,
    						"%s,%s Value %f is out of range %f,%f for "
    						"parameter %s\n",
    						driverName,
							functionName,
							value,
							constraint->minimum,
							constraint->maximum,
							paramString);
        			value = (int)constraint->maximum;
        		}
        		largeValue = (pi64s)value;
                error = Picam_SetParameterLargeIntegerValue(currentCameraHandle,
                        picamParameter, largeValue);
        		if (error != PicamError_None) {
                    Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                            &errorString);
                    asynPrint(pasynUser, ASYN_TRACE_ERROR,
                            "%s:%s error writing %d to  %s \n"
                            "Reason %s and not out of range\n",
                            driverName,
                            functionName,
                            value,
                            paramString,
                            errorString);
                    Picam_DestroyString(errorString);
                    return asynError;
        		}
                Picam_DestroyString(paramString);
        	}
        }
        if (error != PicamError_None) {
            Picam_GetEnumerationString(PicamEnumeratedType_Error, error,
                    &errorString);
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s error writing %d to %s\n"
                    "Reason %s\n",
                    driverName,
                    functionName,
                    largeValue,
                    paramString,
                    errorString);
            Picam_DestroyString(errorString);
            return asynError;
        }

    }
    return (asynStatus)status;
}

/**
 * Write an int 32 value when the constraint type is a collection.  In this
 * case the output will be a string associated with the collection enum
 * value.
 */
asynStatus ADPICam::piWriteInt32CollectionType(asynUser *pasynUser,
        epicsInt32 value,
        int driverParameter,
        PicamParameter picamParameter){
    const char *functionName = "piWriteInt32CollectionType";
    int status = asynSuccess;
    PicamValueType valType;
    PicamError error;

    error = Picam_GetParameterValueType(currentCameraHandle,
            picamParameter,
            &valType);
    if (error != PicamError_None) {
        // TODO
        return asynError;
    }
    if (valType == PicamValueType_Boolean) {
        error = Picam_SetParameterIntegerValue(currentCameraHandle,
                picamParameter, value);
        if (error != PicamError_None) {
            //TODO
            return asynError;
        }
    }
    if (valType == PicamValueType_Integer) {
        error = Picam_SetParameterIntegerValue(currentCameraHandle,
                picamParameter, value);
        if (error != PicamError_None) {
            //TODO
            return asynError;
        }
    }
    if (valType == PicamValueType_LargeInteger) {
        pi64s largeVal = value;
        error = Picam_SetParameterLargeIntegerValue(currentCameraHandle,
                picamParameter, largeVal);
        if (error != PicamError_None) {
            //TODO
            return asynError;
        }
    }
    else if (valType == PicamValueType_FloatingPoint){
        const PicamCollectionConstraint* paramCollection;
        error = Picam_GetParameterCollectionConstraint(currentCameraHandle,
                picamParameter, PicamConstraintCategory_Capable,
                &paramCollection);
        if (error == PicamError_None) {
            Picam_SetParameterFloatingPointValue(currentCameraHandle,
                    picamParameter,
                    paramCollection->values_array[value]);
        } else {
            const char *paramString;
            Picam_GetEnumerationString(PicamEnumeratedType_Parameter,
                    picamParameter,
                    &paramString);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s No collection values available for %s list",
                    paramString);
            Picam_DestroyString(paramString);
        }

    }
    else if (valType == PicamValueType_Enumeration) {
        error = Picam_SetParameterIntegerValue(currentCameraHandle,
                picamParameter, value);
        if (error != PicamError_None) {
            //TODO
            return asynError;
        }
    }
    return (asynStatus)status;
}


/**
 * Callback when a new Image event is seen.  Call the driver's method
 * piHanfleNewImageTask
 */
static void piHandleNewImageTaskC(void *drvPvt)
{
    ADPICam *pPvt = (ADPICam *)drvPvt;

    pPvt->piHandleNewImageTask();
}

/**
 * Handler class for recieving new images.  This runs in a thread separate
 * from the picam driver thread to avoid collisions.  Acquisition in the
 * picam thread will signal this thread as soon as possible when new images
 * are seen.
 */
void ADPICam::piHandleNewImageTask(void)
{
    const char * functionName = "piHandleNewImageTask";
    int imageMode;
    int imagesCounter;
    int numImages;
    int arrayCounter;
    int arrayCallbacks;
    NDArrayInfo arrayInfo;
    epicsTimeStamp currentTime;
    PicamError error;
    int useDriverTimestamps;
    int useFrameTracking;
    int trackFrames;
    int frameTrackingBitDepth;
    pi64s timeStampValue;
    pi64s *pTimeStampValue;
    pi64s frameValue;
    pi64s *pFrameValue;
    int timeStampBitDepth;
    int timeStampResolution;
    int frameSize;
    int numTimeStamps;

  while (1) {
    epicsEventWait(piHandleNewImageEvent);
	getIntegerParam(PICAM_TimeStamps, &useDriverTimestamps);
	getIntegerParam(PICAM_TrackFrames, &useFrameTracking);
    	if (acqStatusErrors == PicamAcquisitionErrorsMask_None) {
            if (acqStatusRunning ||
                    (!acqStatusRunning && (acqAvailableReadoutCount != 0) )) {
                getIntegerParam(ADImageMode, &imageMode);
                getIntegerParam(ADNumImages, &numImages);
                getIntegerParam(ADNumImagesCounter, &imagesCounter);
                imagesCounter++;
                lock();
                setIntegerParam(ADNumImagesCounter, imagesCounter);
                unlock();
                if ((imageMode == ADImageMultiple)
                        && (imagesCounter >= numImages)) {
                    piAcquireStop();
                }
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                        "Acquire, Running %s, errors %d, rate %f, "
                		"availableDataCount %d\n",
                        acqStatusRunning ? "True" : "false", acqStatusErrors,
                        acqStatusReadoutRate, acqAvailableReadoutCount);
                /* Update the image */
                /* First release the copy that we held onto last time */
                if (this->pArrays[0])
                    this->pArrays[0]->release();

                getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
                if (arrayCallbacks) {
                    /* Allocate a new array */
                    this->pArrays[0] = pNDArrayPool->alloc(2, imageDims,
                            imageDataType, 0,
                            NULL);
                    if (this->pArrays[0] == NULL) {
                        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                                "%s:%s error allocating buffer\n", driverName,
                                functionName);
                        return;
                    }
                    if (acqStatusErrors != PicamAcquisitionErrorsMask_None) {
                        const char *acqStatusErrorString;
                        Picam_GetEnumerationString(
                                PicamEnumeratedType_AcquisitionErrorsMask,
                                acqStatusErrors, &acqStatusErrorString);
                        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                                "%s:%s Error found during acquisition: %s",
                                driverName, functionName, acqStatusErrorString);
                        Picam_DestroyString(acqStatusErrorString);
                    }
                    pibln overran;
                    error = PicamAdvanced_HasAcquisitionBufferOverrun(
                            currentDeviceHandle, &overran);
                    if (overran) {
                        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                                "%s:%s Overrun in acquisition buffer",
                                driverName, functionName);
                    }
                    pImage = this->pArrays[0];
                    pImage->getInfo(&arrayInfo);
                    // Copy data from the input to the output
                    dataLock.lock();
                    memcpy(pImage->pData, acqAvailableInitialReadout,
                            arrayInfo.totalBytes);
                    dataLock.unlock();
                    getIntegerParam(NDArrayCounter, &arrayCounter);
                    arrayCounter++;
                    lock();
                    setIntegerParam(NDArrayCounter, arrayCounter);
                    unlock();
                    // Get timestamp from the driver if requested
                    getIntegerParam(PICAM_TimeStampBitDepth,
                			&timeStampBitDepth);
                	getIntegerParam(PICAM_TimeStampResolution,
                			&timeStampResolution);
                	Picam_GetParameterIntegerValue(currentCameraHandle,
                			PicamParameter_FrameSize,
							&frameSize);
                    if (!useDriverTimestamps){
						epicsTimeGetCurrent(&currentTime);
						pImage->timeStamp = currentTime.secPastEpoch
								+ currentTime.nsec / 1.e9;
						updateTimeStamp(&pImage->epicsTS);
                    }
                    else {
                    	pTimeStampValue =
                    			(pi64s*) ((pibyte *)acqAvailableInitialReadout
                    					+ frameSize);
                    	timeStampValue = *pTimeStampValue;
                    	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    			"%s%s TimeStamp %d  Res %d frame size %d timestamp %f\n",
								driverName,
								functionName,
								timeStampValue,
								timeStampResolution,
								frameSize,
								(double)timeStampValue /(double)timeStampResolution);
                    	pImage->timeStamp = (double)timeStampValue /
                    			(double)timeStampResolution;
						updateTimeStamp(&pImage->epicsTS);
                    }
                    // use frame tracking for UniqueID if requested
                    if (!useFrameTracking) {
                    	pImage->uniqueId = arrayCounter;
                    }
                    else {
                    	asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    			"%s:%s  TimeStamps 0X%X\n",
								driverName,
								functionName,
								useDriverTimestamps);
                    	// Frame tracking info follows data and time stamps.
                    	// need to determine the correct number of time stamps
                    	// to skip
                    	if ((useDriverTimestamps ==
                    			PicamTimeStampsMask_None)) {
                    		numTimeStamps = 0;
                    	}
                    	else if ((useDriverTimestamps ==
									PicamTimeStampsMask_ExposureStarted) ||
                    		(useDriverTimestamps ==
                    				PicamTimeStampsMask_ExposureEnded) ) {
                    		numTimeStamps = 1;
                    	}
                    	else  {
                    		numTimeStamps = 2;
                    	}
                    	getIntegerParam(PICAM_FrameTrackingBitDepth,
                    			&frameTrackingBitDepth);
                    	switch (frameTrackingBitDepth){
                    	case 64:
                        	pFrameValue =
                        			(pi64s*) ((pibyte *)acqAvailableInitialReadout
                        			+ frameSize
									+ (numTimeStamps * timeStampBitDepth/8));
                        	asynPrint (pasynUserSelf, ASYN_TRACE_FLOW,
                        			"%s%s Frame tracking bit depth %d"
                        			" timeStampBitDepth %d frameValue %d "
                        			" readout count %d\n",
    								driverName,
    								functionName,
    								frameTrackingBitDepth,
    								timeStampBitDepth,
									*pFrameValue,
									acqAvailableReadoutCount);
                        	pImage->uniqueId = (int)(*pFrameValue);
                        	break;
                    	}
                    }

                    /* Get attributes that have been defined for this driver */
                    getAttributes(pImage->pAttributeList);

                    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                            "%s:%s: calling imageDataCallback\n", driverName,
                            functionName);

                    doCallbacksGenericPointer(pImage, NDArrayData, 0);
                }

            }

            else if (!(acqStatusRunning) && acqAvailableReadoutCount == 0) {
                const char *errorMaskString;
                Picam_GetEnumerationString(
                		PicamEnumeratedType_AcquisitionErrorsMask,
                        acqStatusErrors,
						&errorMaskString);
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "Acquire, Running %s, errors %d, rate %f\n",
                        acqStatusRunning ? "True":"false",
                                errorMaskString,
                                acqStatusReadoutRate
                                );
                Picam_DestroyString(errorMaskString);
                piAcquireStop();
            }
    }
    else if (acqStatusErrors != PicamAcquisitionErrorsMask_None) {
        const char *errorMaskString;
        Picam_GetEnumerationString(PicamEnumeratedType_AcquisitionErrorsMask,
                acqStatusErrors, &errorMaskString);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "Acquire, Running %s, errors %d, rate %f\n",
                acqStatusRunning ? "True":"false",
                        errorMaskString,
                        acqStatusReadoutRate
                        );
        Picam_DestroyString(errorMaskString);
        piAcquireStop();
    }
    unlock();
    callParamCallbacks();
  }
}

/* Code for iocsh registration */

/* PICamConfig */
static const iocshArg PICamConfigArg0 = { "Port name", iocshArgString };
static const iocshArg PICamConfigArg1 = { "maxBuffers", iocshArgInt };
static const iocshArg PICamConfigArg2 = { "maxMemory", iocshArgInt };
static const iocshArg PICamConfigArg3 = { "priority", iocshArgInt };
static const iocshArg PICamConfigArg4 = { "stackSize", iocshArgInt };
static const iocshArg * const PICamConfigArgs[] = { &PICamConfigArg0,
        &PICamConfigArg1, &PICamConfigArg2, &PICamConfigArg3, &PICamConfigArg4 };

static const iocshFuncDef configPICam = { "PICamConfig", 5, PICamConfigArgs };

static const iocshArg PICamAddDemoCamArg0 =
        { "Demo Camera name", iocshArgString };
static const iocshArg * const PICamAddDemoCamArgs[] = { &PICamAddDemoCamArg0 };

static const iocshFuncDef addDemoCamPICam = { "PICamAddDemoCamera", 1,
        PICamAddDemoCamArgs };

static void configPICamCallFunc(const iocshArgBuf *args) {
    PICamConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival,
            args[4].ival);
}

static void addDemoCamPICamCallFunc(const iocshArgBuf *args) {
    PICamAddDemoCamera(args[0].sval);
}

static void PICamRegister(void) {
    iocshRegister(&configPICam, configPICamCallFunc);
    iocshRegister(&addDemoCamPICam, addDemoCamPICamCallFunc);
}

extern "C" {
epicsExportRegistrar(PICamRegister);
}
