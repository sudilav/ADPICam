/* PICam.h
 *
 * This is an areaDetector driver for cameras that communicate
 * with the Priceton Instruments PICAM driver library
 *
 */
#ifndef ADPICAM_H
#define ADPICAM_H

#include <vector>
#include <stdlib.h>

#include <iocsh.h>

#include <epicsString.h>
#include <epicsEvent.h>
#include <epicsThread.h>

#include "ADDriver.h"

#include "picam_advanced.h"


class epicsShareClass ADPICam: public ADDriver {
public:
    static const char *notAvailable;
    static const char *driverName;

    ADPICam(const char *portName, int maxBuffers, size_t maxMemory,
            int priority, int stackSize);
    ~ADPICam();
    /* These are the methods that we override from ADDriver */
    virtual asynStatus readEnum(asynUser *pasynUser, char *strings[],
            int values[], int severities[], size_t nElements, size_t *nIn);
    static PicamError PIL_CALL piAcquistionUpdated(
            PicamHandle device,
            const PicamAvailableData* available,
            const PicamAcquisitionStatus *status);
    static asynStatus piAddDemoCamera(const char *demoCameraName);
    static PicamError PIL_CALL piCameraDiscovered(
            const PicamCameraID *id,
            PicamHandle device,
            PicamDiscoveryAction action);
    asynStatus piHandleAcquisitionUpdated(
            PicamHandle device,
            const PicamAvailableData *available,
            const PicamAcquisitionStatus *acqStatus);
    asynStatus piHandleCameraDiscovery(const PicamCameraID *id,
            PicamHandle device, PicamDiscoveryAction);
    asynStatus piHandleParameterRelevanceChanged(PicamHandle camera,
            PicamParameter parameter, pibln relevant);
    asynStatus piHandleParameterIntegerValueChanged(PicamHandle camera,
            PicamParameter parameter, piint value);
    asynStatus piHandleParameterLargeIntegerValueChanged(PicamHandle camera,
            PicamParameter parameter, pi64s value);
    asynStatus piHandleParameterFloatingPointValueChanged(PicamHandle camera,
            PicamParameter parameter, piflt value);
    asynStatus piHandleParameterRoisValueChanged(PicamHandle camera,
            PicamParameter parameter, const PicamRois *value);
    asynStatus piHandleParameterPulseValueChanged(PicamHandle camera,
            PicamParameter parameter, const PicamPulse *value);
    asynStatus piHandleParameterModulationsValueChanged(PicamHandle camera,
            PicamParameter parameter, const PicamModulations *value);
    asynStatus piLoadAvailableCameraIDs();
    asynStatus piPrintRoisConstraints();
    static PicamError PIL_CALL piParameterFloatingPointValueChanged(
            PicamHandle camera,
            PicamParameter parameter,
            piflt value );
    static PicamError PIL_CALL piParameterIntegerValueChanged(
            PicamHandle camera,
            PicamParameter parameter,
            piint value );
    static PicamError PIL_CALL piParameterLargeIntegerValueChanged(
            PicamHandle camera,
            PicamParameter parameter,
            pi64s value );
    static PicamError PIL_CALL piParameterModulationsValueChanged(
            PicamHandle camera,
            PicamParameter parameter,
            const PicamModulations *value );
    static PicamError PIL_CALL piParameterPulseValueChanged(
            PicamHandle camera,
            PicamParameter parameter,
            const PicamPulse *value );
    static PicamError PIL_CALL piParameterRelevanceChanged(
            PicamHandle camera,
            PicamParameter parameter,
            pibln relevent );
    static PicamError PIL_CALL piParameterRoisValueChanged(
            PicamHandle camera,
            PicamParameter parameter,
            const PicamRois *value );
    void piHandleNewImageTask(void);
    void report(FILE *fp, int details);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value,
                                        size_t nChars, size_t *nActual,
										int *eomReason);
protected:

    int PICAM_VersionNumber;
#define PICAM_FIRST_PARAM PICAM_VersionNumber
    int PICAM_AvailableCameras;
    int PICAM_CameraInterface;
    int PICAM_SensorName;
    int PICAM_SerialNumber;
    int PICAM_FirmwareRevision;

    int PICAM_UnavailableCameras;
    int PICAM_CameraInterfaceUnavailable;
    int PICAM_SensorNameUnavailable;
    int PICAM_SerialNumberUnavailable;
    int PICAM_FirmwareRevisionUnavailable;

    //Shutter Timing
    int PICAM_ShutterClosingDelay;
    int PICAM_ShutterDelayResolution;
    int PICAM_ShutterOpeningDelay;
    int PICAM_ShutterTimingMode;

    // Intensifier
    int PICAM_BracketGating;
    int PICAM_CustomModulationSequence;
    int PICAM_DifEndingGate;
    int PICAM_EMIccdGain;
    int PICAM_EMIccdGainControlMode;
    int PICAM_EnableIntensifier;
    int PICAM_EnableModulation;
    int PICAM_GatingMode;
    int PICAM_GatingSpeed;
    int PICAM_IntensifierDiameter;
    int PICAM_IntensifierGain;
    int PICAM_IntensifierOptions;
    int PICAM_IntensifierStatus;
    int PICAM_ModulationDuration;
    int PICAM_ModulationFrequency;
    int PICAM_PhosphorDecayDelay;
    int PICAM_PhosphorDecayDelayResolution;
    int PICAM_PhosphorType;
    int PICAM_PhotocathodeSensitivity;
    int PICAM_RepetitiveGate;
    int PICAM_RepetitiveModulation;
    int PICAM_SequentialStartingModulationPhase;
    int PICAM_SequentialEndingModulationPhase;
    int PICAM_SequentialEndingGate;
    int PICAM_SequentialGateStepCount;
    int PICAM_SequentialGateStepIterations;
    int PICAM_SequentialStartingGate;

    //ADC
    int PICAM_AdcAnalogGain;
    int PICAM_AdcBitDepth;
    int PICAM_AdcEMGain;
    int PICAM_AdcQuality;
    int PICAM_AdcSpeed;
    int PICAM_CorrectPixelBias;

    //Hardware I/O
    int PICAM_AuxOutput;
    int PICAM_EnableModulationOutputSignal;
    int PICAM_ModulationOutputSignalFrequency;
    int PICAM_ModulationOutputSignalAmplitude;
    int PICAM_EnableSyncMaster;
    int PICAM_InvertOutputSignal;
    int PICAM_OutputSignal;
    int PICAM_SyncMaster2Delay;
    int PICAM_TriggerCoupling;
    int PICAM_TriggerDetermination;
    int PICAM_TriggerFrequency;
    int PICAM_TriggerSource;
    int PICAM_TriggerTermination;
    int PICAM_TriggerThreshold;

    //ReadoutControl
    int PICAM_Accumulations;
    int PICAM_EnableNondestructiveReadout;
    int PICAM_KineticsWindowHeight;
    int PICAM_NondestructiveReadoutPeriod;
    int PICAM_ReadoutControlMode;
    int PICAM_ReadoutOrientation;
    int PICAM_ReadoutPortCount;
    int PICAM_ReadoutTimeCalc;
    int PICAM_VerticalShiftRate;

    //DataAcquisition
    int PICAM_DisableDataFormatting;
    int PICAM_ExactReadoutCountMax;
    int PICAM_FrameRateCalc;
    int PICAM_FramesPerReadout;
    int PICAM_FrameStride;
    int PICAM_FrameTrackingBitDepth;
    int PICAM_GateTracking;
    int PICAM_GateTrackingBitDepth;
    int PICAM_ModulationTracking;
    int PICAM_ModulationTrackingBitDepth;
    int PICAM_NormalizeOrientation;
    int PICAM_OnlineReadoutRateCalc;
    int PICAM_Orientation;
    int PICAM_PhotonDetectionMode;
    int PICAM_PhotonDetectionThreshold;
    int PICAM_PixelBitDepth;
    int PICAM_PixelFormat;
    int PICAM_ReadoutCount;
    int PICAM_ReadoutRateCalc;
    int PICAM_ReadoutStride;
    int PICAM_TimeStampBitDepth;
    int PICAM_TimeStampResolution;
    int PICAM_TimeStamps;
    int PICAM_TrackFrames;

    //Sensor Information
    int PICAM_CcdCharacteristics;
    int PICAM_PixelGapHeight;
    int PICAM_PixelGapWidth;
    int PICAM_PixelHeight;
    int PICAM_PixelWidth;
    int PICAM_SensorActiveBottomMargin;
    int PICAM_SensorActiveHeight;
    int PICAM_SensorActiveLeftMargin;
    int PICAM_SensorActiveRightMargin;
    int PICAM_SensorActiveTopMargin;
    int PICAM_SensorActiveWidth;
    int PICAM_SensorMaskedBottomMargin;
    int PICAM_SensorMaskedHeight;
    int PICAM_SensorMaskedTopMargin;
    int PICAM_SensorSecondaryActiveHeight;
    int PICAM_SensorSecondaryMaskedHeight;
    int PICAM_SensorType;

    //SensorLayout
    int PICAM_ActiveBottomMargin;
    int PICAM_ActiveHeight;
    int PICAM_ActiveLeftMargin;
    int PICAM_ActiveRightMargin;
    int PICAM_ActiveTopMargin;
    int PICAM_ActiveWidth;
    int PICAM_MaskedBottomMargin;
    int PICAM_MaskedHeight;
    int PICAM_MaskedTopMargin;
    int PICAM_SecondaryActiveHeight;
    int PICAM_SecondaryMaskedHeight;

    //Sensor Cleaning
    int PICAM_CleanBeforeExposure;
    int PICAM_CleanCycleCount;
    int PICAM_CleanCycleHeight;
    int PICAM_CleanSectionFinalHeight;
    int PICAM_CleanSectionFinalHeightCount;
    int PICAM_CleanSerialRegister;
    int PICAM_CleanUntilTrigger;

    //Sensor Temperature
    int PICAM_DisableCoolingFan;
    int PICAM_EnableSensorWindowHeater;
    int PICAM_SensorTemperatureStatus;

    //Display Aids
    int PICAM_EnableROIMinXInput;
    int PICAM_EnableROISizeXInput;
    int PICAM_EnableROIMinYInput;
    int PICAM_EnableROISizeYInput;

    // Camera Parameter exists for this detector
    int PICAM_ExposureTimeExists;
    int PICAM_ShutterClosingDelayExists;
    int PICAM_ShutterDelayResolutionExists;
    int PICAM_ShutterOpeningDelayExists;
    int PICAM_ShutterTimingModeExists;
    int PICAM_BracketGatingExists;
    int PICAM_CustomModulationSequenceExists;
    int PICAM_DifEndingGateExists;
    int PICAM_DifStartingGateExists;
    int PICAM_EMIccdGainExists;
    int PICAM_EMIccdGainControlModeExists;
    int PICAM_EnableIntensifierExists;
    int PICAM_EnableModulationExists;
    int PICAM_GatingModeExists;
    int PICAM_GatingSpeedExists;
    int PICAM_IntensifierDiameterExists;
    int PICAM_IntensifierGainExists;
    int PICAM_IntensifierOptionsExists;
    int PICAM_IntensifierStatusExists;
    int PICAM_ModulationDurationExists;
    int PICAM_ModulationFrequencyExists;
    int PICAM_PhosphorDecayDelayExists;
    int PICAM_PhosphorDecayDelayResolutionExists;
    int PICAM_PhosphorTypeExists;
    int PICAM_PhotocathodeSensitivityExists;
    int PICAM_RepetitiveGateExists;
    int PICAM_RepetitiveModulationPhaseExists;
    int PICAM_SequentialStartingModulationPhaseExists;
    int PICAM_SequentialEndingModulationPhaseExists;
    int PICAM_SequentialEndingGateExists;
    int PICAM_SequentialGateStepCountExists;
    int PICAM_SequentialGateStepIterationsExists;
    int PICAM_SequentialStartingGateExists;
    int PICAM_AdcAnalogGainExists;
    int PICAM_AdcBitDepthExists;
    int PICAM_AdcEMGainExists;
    int PICAM_AdcQualityExists;
    int PICAM_AdcSpeedExists;
    int PICAM_CorrectPixelBiasExists;
    int PICAM_AuxOutputExists;
    int PICAM_EnableModulationOutputSignalExists;
    int PICAM_EnableModulationOutputSignalFrequencyExists;
    int PICAM_EnableModulationOutputSignalAmplitudeExists;
    int PICAM_EnableSyncMasterExists;
    int PICAM_InvertOutputSignalExists;
    int PICAM_OutputSignalExists;
    int PICAM_SyncMaster2DelayExists;
    int PICAM_TriggerCouplingExists;
    int PICAM_TriggerDeterminationExists;
    int PICAM_TriggerFrequencyExists;
    int PICAM_TriggerResponseExists;
    int PICAM_TriggerSourceExists;
    int PICAM_TriggerTerminationExists;
    int PICAM_TriggerThresholdExists;
    int PICAM_AccumulationsExists;
    int PICAM_EnableNondestructiveReadoutExists;
    int PICAM_KineticsWindowHeightExists;
    int PICAM_NondestructiveReadoutPeriodExists;
    int PICAM_ReadoutControlModeExists;
    int PICAM_ReadoutOrientationExists;
    int PICAM_ReadoutPortCountExists;
    int PICAM_ReadoutTimeCalculationExists;
    int PICAM_VerticalShiftRateExists;
    int PICAM_DisableDataFormattingExists;
    int PICAM_ExactReadoutCountMaximumExists;
    int PICAM_FrameRateCalculationExists;
    int PICAM_FrameSizeExists;
    int PICAM_FramesPerReadoutExists;
    int PICAM_FrameStrideExists;
    int PICAM_FrameTrackingBitDepthExists;
    int PICAM_GateTrackingExists;
    int PICAM_GateTrackingBitDepthExists;
    int PICAM_ModulationTrackingExists;
    int PICAM_ModulationTrackingBitDepthExists;
    int PICAM_NormalizeOrientationExists;
    int PICAM_OnlineReadoutRateCalculationExists;
    int PICAM_OrientationExists;
    int PICAM_PhotonDetectionModeExists;
    int PICAM_PhotonDetectionThresholdExists;
    int PICAM_PixelBitDepthExists;
    int PICAM_PixelFormatExists;
    int PICAM_ReadoutCountExists;
    int PICAM_ReadoutRateCalculationExists;
    int PICAM_ReadoutStrideExists;
    int PICAM_RoisExists;
    int PICAM_TimeStampBitDepthExists;
    int PICAM_TimeStampResolutionExists;
    int PICAM_TimeStampsExists;
    int PICAM_TrackFramesExists;
    int PICAM_CcdCharacteristicsExists;
    int PICAM_PixelGapHeightExists;
    int PICAM_PixelGapWidthExists;
    int PICAM_PixelHeightExists;
    int PICAM_PixelWidthExists;
    int PICAM_SensorActiveBottomMarginExists;
    int PICAM_SensorActiveHeightExists;
    int PICAM_SensorActiveLeftMarginExists;
    int PICAM_SensorActiveRightMarginExists;
    int PICAM_SensorActiveTopMarginExists;
    int PICAM_SensorActiveWidthExists;
    int PICAM_SensorMaskedBottomMarginExists;
    int PICAM_SensorMaskedHeightExists;
    int PICAM_SensorMaskedTopMarginExists;
    int PICAM_SensorSecondaryActiveHeightExists;
    int PICAM_SensorSecondaryMaskedHeightExists;
    int PICAM_SensorTypeExists;
    int PICAM_ActiveBottomMarginExists;
    int PICAM_ActiveHeightExists;
    int PICAM_ActiveLeftMarginExists;
    int PICAM_ActiveRightMarginExists;
    int PICAM_ActiveTopMarginExists;
    int PICAM_ActiveWidthExists;
    int PICAM_MaskedBottomMarginExists;
    int PICAM_MaskedHeightExists;
    int PICAM_MaskedTopMarginExists;
    int PICAM_SecondaryActiveHeightExists;
    int PICAM_SecondaryMaskedHeightExists;
    int PICAM_CleanBeforeExposureExists;
    int PICAM_CleanCycleCountExists;
    int PICAM_CleanCycleHeightExists;
    int PICAM_CleanSectionFinalHeightExists;
    int PICAM_CleanSectionFinalHeightCountExists;
    int PICAM_CleanSerialRegisterExists;
    int PICAM_CleanUntilTriggerExists;
    int PICAM_DisableCoolingFanExists;
    int PICAM_EnableSensorWindowHeaterExists;
    int PICAM_SensorTemperatureReadingExists;
    int PICAM_SensorTemperatureSetPointExists;
    int PICAM_SensorTemperatureStatusExists;

    int PICAM_ExposureTimeRelevant;
    int PICAM_ShutterClosingDelayRelevant;
    int PICAM_ShutterDelayResolutionRelevant;
    int PICAM_ShutterOpeningDelayRelevant;
    int PICAM_ShutterTimingModeRelevant;
    int PICAM_BracketGatingRelevant;
    int PICAM_CustomModulationSequenceRelevant;
    int PICAM_DifEndingGateRelevant;
    int PICAM_DifStartingGateRelevant;
    int PICAM_EMIccdGainRelevant;
    int PICAM_EMIccdGainControlModeRelevant;
    int PICAM_EnableIntensifierRelevant;
    int PICAM_EnableModulationRelevant;
    int PICAM_GatingModeRelevant;
    int PICAM_GatingSpeedRelevant;
    int PICAM_IntensifierDiameterRelevant;
    int PICAM_IntensifierGainRelevant;
    int PICAM_IntensifierOptionsRelevant;
    int PICAM_IntensifierStatusRelevant;
    int PICAM_ModulationDurationRelevant;
    int PICAM_ModulationFrequencyRelevant;
    int PICAM_PhosphorDecayDelayRelevant;
    int PICAM_PhosphorDecayDelayResolutionRelevant;
    int PICAM_PhosphorTypeRelevant;
    int PICAM_PhotocathodeSensitivityRelevant;
    int PICAM_RepetitiveGateRelevant;
    int PICAM_RepetitiveModulationPhaseRelevant;
    int PICAM_SequentialStartingModulationPhaseRelevant;
    int PICAM_SequentialEndingModulationPhaseRelevant;
    int PICAM_SequentialEndingGateRelevant;
    int PICAM_SequentialGateStepCountRelevant;
    int PICAM_SequentialGateStepIterationsRelevant;
    int PICAM_SequentialStartingGateRelevant;
    int PICAM_AdcAnalogGainRelevant;
    int PICAM_AdcBitDepthRelevant;
    int PICAM_AdcEMGainRelevant;
    int PICAM_AdcQualityRelevant;
    int PICAM_AdcSpeedRelevant;
    int PICAM_CorrectPixelBiasRelevant;
    int PICAM_AuxOutputRelevant;
    int PICAM_EnableModulationOutputSignalRelevant;
    int PICAM_EnableModulationOutputSignalFrequencyRelevant;
    int PICAM_EnableModulationOutputSignalAmplitudeRelevant;
    int PICAM_EnableSyncMasterRelevant;
    int PICAM_InvertOutputSignalRelevant;
    int PICAM_OutputSignalRelevant;
    int PICAM_SyncMaster2DelayRelevant;
    int PICAM_TriggerCouplingRelevant;
    int PICAM_TriggerDeterminationRelevant;
    int PICAM_TriggerFrequencyRelevant;
    int PICAM_TriggerResponseRelevant;
    int PICAM_TriggerSourceRelevant;
    int PICAM_TriggerTerminationRelevant;
    int PICAM_TriggerThresholdRelevant;
    int PICAM_AccumulationsRelevant;
    int PICAM_EnableNondestructiveReadoutRelevant;
    int PICAM_KineticsWindowHeightRelevant;
    int PICAM_NondestructiveReadoutPeriodRelevant;
    int PICAM_ReadoutControlModeRelevant;
    int PICAM_ReadoutOrientationRelevant;
    int PICAM_ReadoutPortCountRelevant;
    int PICAM_ReadoutTimeCalculationRelevant;
    int PICAM_VerticalShiftRateRelevant;
    int PICAM_DisableDataFormattingRelevant;
    int PICAM_ExactReadoutCountMaximumRelevant;
    int PICAM_FrameRateCalculationRelevant;
    int PICAM_FrameSizeRelevant;
    int PICAM_FramesPerReadoutRelevant;
    int PICAM_FrameStrideRelevant;
    int PICAM_FrameTrackingBitDepthRelevant;
    int PICAM_GateTrackingRelevant;
    int PICAM_GateTrackingBitDepthRelevant;
    int PICAM_ModulationTrackingRelevant;
    int PICAM_ModulationTrackingBitDepthRelevant;
    int PICAM_NormalizeOrientationRelevant;
    int PICAM_OnlineReadoutRateCalculationRelevant;
    int PICAM_OrientationRelevant;
    int PICAM_PhotonDetectionModeRelevant;
    int PICAM_PhotonDetectionThresholdRelevant;
    int PICAM_PixelBitDepthRelevant;
    int PICAM_PixelFormatRelevant;
    int PICAM_ReadoutCountRelevant;
    int PICAM_ReadoutRateCalculationRelevant;
    int PICAM_ReadoutStrideRelevant;
    int PICAM_RoisRelevant;
    int PICAM_TimeStampBitDepthRelevant;
    int PICAM_TimeStampResolutionRelevant;
    int PICAM_TimeStampsRelevant;
    int PICAM_TrackFramesRelevant;
    int PICAM_CcdCharacteristicsRelevant;
    int PICAM_PixelGapHeightRelevant;
    int PICAM_PixelGapWidthRelevant;
    int PICAM_PixelHeightRelevant;
    int PICAM_PixelWidthRelevant;
    int PICAM_SensorActiveBottomMarginRelevant;
    int PICAM_SensorActiveHeightRelevant;
    int PICAM_SensorActiveLeftMarginRelevant;
    int PICAM_SensorActiveRightMarginRelevant;
    int PICAM_SensorActiveTopMarginRelevant;
    int PICAM_SensorActiveWidthRelevant;
    int PICAM_SensorMaskedBottomMarginRelevant;
    int PICAM_SensorMaskedHeightRelevant;
    int PICAM_SensorMaskedTopMarginRelevant;
    int PICAM_SensorSecondaryActiveHeightRelevant;
    int PICAM_SensorSecondaryMaskedHeightRelevant;
    int PICAM_SensorTypeRelevant;
    int PICAM_ActiveBottomMarginRelevant;
    int PICAM_ActiveHeightRelevant;
    int PICAM_ActiveLeftMarginRelevant;
    int PICAM_ActiveRightMarginRelevant;
    int PICAM_ActiveTopMarginRelevant;
    int PICAM_ActiveWidthRelevant;
    int PICAM_MaskedBottomMarginRelevant;
    int PICAM_MaskedHeightRelevant;
    int PICAM_MaskedTopMarginRelevant;
    int PICAM_SecondaryActiveHeightRelevant;
    int PICAM_SecondaryMaskedHeightRelevant;
    int PICAM_CleanBeforeExposureRelevant;
    int PICAM_CleanCycleCountRelevant;
    int PICAM_CleanCycleHeightRelevant;
    int PICAM_CleanSectionFinalHeightRelevant;
    int PICAM_CleanSectionFinalHeightCountRelevant;
    int PICAM_CleanSerialRegisterRelevant;
    int PICAM_CleanUntilTriggerRelevant;
    int PICAM_DisableCoolingFanRelevant;
    int PICAM_EnableSensorWindowHeaterRelevant;
    int PICAM_SensorTemperatureReadingRelevant;
    int PICAM_SensorTemperatureSetPointRelevant;
    int PICAM_SensorTemperatureStatusRelevant;
#define PICAM_LAST_PARAM PICAM_SensorTemperatureStatusRelevant

private:
    std::vector<pibyte> buffer_;
    PicamHandle currentCameraHandle;
    PicamHandle currentDeviceHandle;
    int selectedCameraIndex;
    const PicamCameraID *availableCameraIDs;
    const PicamCameraID *unavailableCameraIDs;
    piint availableCamerasCount;
    piint unavailableCamerasCount;
    NDArray *pImage;
    epicsEventId  piHandleNewImageEvent;
    size_t imageDims[2];
    NDDataType_t  imageDataType;
    pibln acqStatusRunning;
    PicamAcquisitionErrorsMask acqStatusErrors;
    piflt acqStatusReadoutRate;
    void *acqAvailableInitialReadout;
    pi64s acqAvailableReadoutCount;
    epicsMutex dataLock;

    asynStatus initializeDetector();
    asynStatus piAcquireStart();
    asynStatus piAcquireStop();
    asynStatus piClearParameterExists();
    asynStatus piClearParameterRelevance();
    asynStatus piLoadUnavailableCameraIDs();
    int piLookupDriverParameter(PicamParameter picamParameter);
    PicamError piLookupPICamParameter(int driverParameter,
            PicamParameter &parameter);
    asynStatus piRegisterConstraintChangeWatch(PicamHandle cameraHandle);
    asynStatus piRegisterRelevantWatch(PicamHandle cameraHandle);
    asynStatus piRegisterValueChangeWatch(PicamHandle cameraHandle);
    asynStatus piSetParameterExists(asynUser *pasynUser,
            PicamParameter parameter, int exists);
    asynStatus piSetParameterRelevance(asynUser *pasynUser,
            PicamParameter parameter, int relevence);
    asynStatus piSetParameterValuesFromSelectedCamera();
    asynStatus piSetRois(int minX, int minY, int width, int height, int binX,
            int binY);
    asynStatus piSetSelectedCamera(asynUser *pasynUser, int selectedIndex);
    asynStatus piSetSelectedUnavailableCamera(asynUser *pasynUser,
            int selectedIndex);
    asynStatus piUnregisterConstraintChangeWatch(PicamHandle cameraHandle);
    asynStatus piUnregisterRelevantWatch(PicamHandle cameraHandle);
    asynStatus piUnregisterValueChangeWatch(PicamHandle cameraHandle);
    asynStatus piUpdateAvailableCamerasList();
    asynStatus piUpdateParameterExists();
    asynStatus piUpdateParameterListValues(PicamParameter parameter,
            int driverParameter);
    asynStatus piUpdateParameterRelevance();
    asynStatus piUpdateUnavailableCamerasList();
    asynStatus piWriteInt32CollectionType(asynUser *pasynUser,
            epicsInt32 value,
            int driverParameter,
            PicamParameter picamParameter);
    asynStatus piWriteInt32RangeType(asynUser *pasynUser,
            epicsInt32 value,
            int driverParameter,
            PicamParameter picamParameter);
    asynStatus piWriteFloat64RangeType(asynUser *pasynUser,
            epicsFloat64 value,
            int driverParameter,
            PicamParameter picamParameter);

    static ADPICam *ADPICam_Instance;
};

//_____________________________________________________________________________
#define PICAM_VersionNumberString          "PICAM_VERSION_NUMBER"
//Available Camera List
#define PICAM_AvailableCamerasString       "PICAM_AVAILABLE_CAMERAS"
#define PICAM_CameraInterfaceString        "PICAM_CAMERA_INTERFACE"
#define PICAM_SensorNameString             "PICAM_SENSOR_NAME"
#define PICAM_SerialNumberString           "PICAM_SERIAL_NUMBER"
#define PICAM_FirmwareRevisionString       "PICAM_FIRMWARE_REVISION"
//Unavailable Camera List
#define PICAM_UnavailableCamerasString          "PICAM_UNAVAILABLE_CAMERAS"
#define PICAM_CameraInterfaceUnavailableString  "PICAM_CAMERA_INTERFACE_UNAVAILABLE"
#define PICAM_SensorNameUnavailableString       "PICAM_SENSOR_NAME_UNAVAILABLE"
#define PICAM_SerialNumberUnavailableString     "PICAM_SERIAL_NUMBER_UNAVAILABLE"
#define PICAM_FirmwareRevisionUnavailableString "PICAM_FIRMWARE_REVISION_UNAVAILABLE"

//Shutter Timing
#define PICAM_ShutterDelayResolutionString                   "PICAM_SHUTTER_DELAY_RESOLUTION"
#define PICAM_ShutterTimingModeString                        "PICAM_SHUTTER_TIMING_MODE"

// Intensifier
#define PICAM_BracketGatingString                      "PICAM_BRACKET_GATING"
#define PICAM_CustomModulationSequenceString           "PICAM_CUSTOM_MODULATION_SEQUENCE"
#define PICAM_DifEndingGateString                      "PICAM_DIF_ENDING_GATE"
#define PICAM_DifStartingGateString                    "PICAM_DIF_STARTING_GATE"
#define PICAM_EMIccdGainString                         "PICAM_EMI_CCD_GAIN"
#define PICAM_EMIccdGainControlModeString              "PICAM_EMI_CCD_GAIN_CONTROL_MODE"
#define PICAM_EnableIntensifierString                  "PICAM_ENABLE_INTENSIFIER"
#define PICAM_EnableModulationString                   "PICAM_ENABLE_MODULATION"
#define PICAM_GatingModeString                         "PICAM_GATING_MODE"
#define PICAM_GatingSpeedString                        "PICAM_GATING_SPEED"
#define PICAM_IntensifierDiameterString                "PICAM_INTENSIFIER_DIAMETER"
#define PICAM_IntensifierGainString                    "PICAM_INTENSIFIER_GAIN"
#define PICAM_IntensifierOptionsString                 "PICAM_INTENSIFIER_OPTIONS"
#define PICAM_IntensifierStatusString                  "PICAM_INTENSIFIER_STATUS"
#define PICAM_ModulationDurationString                 "PICAM_MODULATION_DURATION"
#define PICAM_ModulationFrequencyString                "PICAM_MODULATION_FREQUENCY"
#define PICAM_PhosphorDecayDelayString                 "PICAM_PHOSPHOR_DECAY_DELAY"
#define PICAM_PhosphorDecayDelayResolutionString       "PICAM_PHOSPHOR_DECAY_DELAY_RESOLUTION"
#define PICAM_PhosphorTypeString                       "PICAM_PHOSPHOR_TYPE"
#define PICAM_PhotocathodeSensitivityString            "PICAM_PHOTOCATHODE_SENSITIVITY"
#define PICAM_RepetitiveGateString                     "PICAM_REPETITIVE_GATE"
#define PICAM_RepetitiveModulationString               "PICAM_REPETITIVE_MODULATION"
#define PICAM_SequentialStartingModulationPhaseString  "PICAM_SEQUENTIAL_STARTING_MODULATION_PHASE"
#define PICAM_SequentialEndingModulationPhaseString    "PICAM_SEQUENTIAL_ENDING_MODULATION_PHASE"
#define PICAM_SequentialEndingGateString               "PICAM_SEQUENTIAL_ENDING_GATE"
#define PICAM_SequentialGateStepCountString            "PICAM_SEQUENTIAL_GATE_STEP_COUNT"
#define PICAM_SequentialGateStepIterationsString       "PICAM_SEQUENTIAL_GATE_STEP_ITERATIONS"
#define PICAM_SequentialStartingGareString             "PICAM_SEQUENTIAL_STARTING_GATE"

//AnalogToDigitalConversion
#define PICAM_AdcAnalogGainString               "PICAM_ADC_ANALOG_GAIN"
#define PICAM_AdcBitDepthString                 "PICAM_ADC_BIT_DEPTH"
#define PICAM_AdcEMGainString                   "PICAM_ADC_EM_GAIN"
#define PICAM_AdcQualityString                  "PICAM_ADC_QUALITY"
#define PICAM_AdcSpeedString                    "PICAM_ADC_SPEED"
#define PICAM_CorrectPixelBiasString            "PICAM_CORRECT_PIXEL_BIAS"
//Hardware I/O
#define PICAM_AuxOutputString                          "PICAM_AUX_OUTPUT_STRING"
#define PICAM_EnableModulationOutputSignalString       "PICAM_ENABLE_MODULATION_OUTPUT_SIGNAL"
#define PICAM_ModulationOutputSignalFrequencyString    "PICAM_MODULATION_OUTPUT_SIGNAL_FREQUENCY"
#define PICAM_ModulationOutputSignalAmplitudeString    "PICAM_MODULATION_OUTPUT_SIGNAL_AMPLITUDE"
#define PICAM_EnableSyncMasterString                   "PICAM_ENABLE_SYNC_MASTER"
#define PICAM_InvertOutputSignalString                 "PICAM_INVERT_OUTPUT_SIGNAL"
#define PICAM_OutputSignalString                       "PICAM_OUTPUT_SIGNAL"
#define PICAM_SyncMaster2DelayString                   "PICAM_SYNC_MASTER2_DELAY"
#define PICAM_TriggerCouplingString                    "PICAM_TRIGGER_COUPLING"
#define PICAM_TriggerDeterminationString               "PICAM_TRIGGER_DETERMINATION"
#define PICAM_TriggerFrequencyString                   "PICAM_TRIGGER_FREQUENCY"
#define PICAM_TriggerSourceString                      "PICAM_TRIGGER_SOURCE"
#define PICAM_TriggerTerminationString                 "PICAM_TRIGGER_TERMINATION"
#define PICAM_TriggerThresholdString                   "PICAM_TRIGGER_THRESHOLD"

//ReadoutControl
#define PICAM_AccumulationsString                  "PICAM_ACCUMULATIONS"
#define PICAM_EnableNondestructiveReadoutString    "PICAM_ENABLE_NONDESTRUCTIVE_READOUT"
#define PICAM_KineticsWindowHeightString           "PICAM_KINETICS_WINDOW_HEIGHT"
#define PICAM_NondestructiveReadoutPeriodString    "PICAM_NONDESTRUCTIVE_READOUT_PERIOD"
#define PICAM_ReadoutControlModeString             "PICAM_READOUT_CONTROL_MODE"
#define PICAM_ReadoutOrientationString             "PICAM_READOUT_ORIENTATION"
#define PICAM_ReadoutPortCountString               "PICAM_READOUT_PORT_COUNT"
#define PICAM_ReadoutTimeCalcString                "PICAM_READOUT_TIME_CALC"
#define PICAM_VerticalShiftRateString              "PICAM_VERTICAL_SHIFT_RATE"

//DataAcquisition
#define PICAM_DisableDataFormattingString        "PICAM_DISABLE_DATA_FORMATTING"
#define PICAM_ExactReadoutCountMaxString         "PICAM_EXACT_READOUT_COUNT_MAX"
#define PICAM_FrameRateCalcString                "PICAM_FRAME_RATE_CALC"
#define PICAM_FramesPerReadoutString             "PICAM_FRAMES_PER_READOUT"
#define PICAM_FrameStrideString                  "PICAM_FRAME_STRIDE"
#define PICAM_FrameTrackingBitDepthString        "PICAM_FRAME_TRACKING_BIT_DEPTH"
#define PICAM_GateTrackingString                 "PICAM_GATE_TRACKING"
#define PICAM_GateTrackingBitDepthString         "PICAM_GATE_TRACKING_BIT_DEPTH"
#define PICAM_ModulationTrackingString           "PICAM_MODULATION_TRACKING"
#define PICAM_ModulationTrackingBitDepthString   "PICAM_MODULATION_TRACKING_BIT_DEPTH"
#define PICAM_NormalizeOrientationString         "PICAM_NORMALIZE_ORIENTATION"
#define PICAM_OnlineReadoutRateCalcString        "PICAM_ONLINE_READOUT_RATE_CALC"
#define PICAM_OrientationString                  "PICAM_ORIENTATION"
#define PICAM_PhotonDetectionModeString          "PICAM_PHOTON_DETECTION_MODE"
#define PICAM_PhotonDetectionThresholdString     "PICAM_PHOTON_DETECTION_THRESHOLD"
#define PICAM_PixelBitDepthString                "PICAM_PIXEL_BIT_DEPTH"
#define PICAM_PixelFormatString                  "PICAM_PIXEL_FORMAT"
#define PICAM_ReadoutCountString                 "PICAM_READOUT_COUNT"
#define PICAM_ReadoutRateCalcString              "PICAM_READOUT_RATE_CALC"
#define PICAM_ReadoutStrideString                "PICAM_READOUT_STRIDE"
#define PICAM_TimeStampBitDepthString            "PICAM_TIME_STAMP_BIT_DEPTH"
#define PICAM_TimeStampResolutionString          "PICAM_TIME_STAMP_RESOLUTION"
#define PICAM_TimeStampsString                   "PICAM_TIME_STAMPS"
#define PICAM_TrackFramesString                  "PICAM_TRACK_FRAMES"

//Sensor Information
#define PICAM_CcdCharacteristicsString           "PICAM_CCD_CHARACTERISTICS"
#define PICAM_PixelGapHeightString               "PICAM_PIXEL_GAP_HEIGHT"
#define PICAM_PixelGapWidthString                "PICAM_PIXEL_GAP_WIDTH"
#define PICAM_PixelHeightString                  "PICAM_PIXEL_HEIGHT"
#define PICAM_PixelWidthString                   "PICAM_PIXEL_WIDTH"
#define PICAM_SensorActiveBottomMarginString     "PICAM_SENSOR_ACTIVE_BOTTOM_MARGIN"
#define PICAM_SensorActiveHeightString           "PICAM_SEMSOR_ACTIVE_HEIGHT"
#define PICAM_SensorActiveLeftMarginString       "PICAM_SENSOR_ACTIVE_LEFT_MARGIN"
#define PICAM_SensorActiveRightMarginString      "PICAM_SENSOR_ACTIVE_RIGHT_MARGIN"
#define PICAM_SensorActiveTopMarginString        "PICAM_SENSOR_ACTIVE_TOP_MARGIN"
#define PICAM_SensorActiveWidthString            "PICAM_SENSOE_ACTIVE_WIDTH"
#define PICAM_SensorMaskedBottomMarginString     "PICAM_SENSOR_MASKED_BOTTOM_MARGIN"
#define PICAM_SensorMaskedHeightString           "PICAM_SENSOR_MASKED_HEIGHT"
#define PICAM_SensorMaskedTopMarginString        "PICAM_SENSOR_MASKED_TOP_MARGIN"
#define PICAM_SensorSecondaryActiveHeightString  "PICAM_SENSOR_SECONDARY_ACTIVE_HEIGHT"
#define PICAM_SensorSecondaryMaskedHeightString  "PICAM_SENSOR_SECONDARY_MASKED_HEIGHT"
#define PICAM_SensorTypeString                   "PICAM_SENSOR_TYPE"

//SensorLayout
#define PICAM_ActiveBottomMarginString           "PICAM_ACTIVE_BOTTOM_MARGIN"
#define PICAM_ActiveHeightString                 "PICAM_ACTIVE_HEIGHT"
#define PICAM_ActiveLeftMarginString             "PICAM_ACTIVE_LEFT_MARGIN"
#define PICAM_ActiveRightMarginString            "PICAM_ACTIVE_RIGHT_MARGIN"
#define PICAM_ActiveTopMarginString              "PICAM_ACTIVE_TOP_MARGIN"
#define PICAM_ActiveWidthString                  "PICAM_ACTIVE_WIDTH"
#define PICAM_MaskedBottomMarginString           "PICAM_MASKED_BOTTOM_MARGIN"
#define PICAM_MaskedHeightString                 "PICAM_MASKED_HEIGHT"
#define PICAM_MaskedTopMarginString             "PICAM_MASKED_TOP_MARGIN"
#define PICAM_SecondaryActiveHeightString        "PICAM_SECONDARY_ACTIVE_HEIGHT"
#define PICAM_SecondaryMaskedHeightString         "PICAM_SECONDARY_MASKED_HEIGHT"
//Sensor Cleaning
#define PICAM_CleanBeforeExposureString          "PICAM_CLEAN_BEFORE_EXPOSURE"
#define PICAM_CleanCycleCountString              "PICAM_CLEAN_CYCLE_COUNT"
#define PICAM_CleanCycleHeightString             "PICAM_CLEAN_CYCLE_HEIGHT"
#define PICAM_CleanSectionFinalHeightString      "PICAM_CLEAN_SECTION_FINAL_HEIGHT"
#define PICAM_CleanSectionFinalHeightCountString "PICAM_CLEAN_SECTION_FINAL_HEIGHT_COUNT"
#define PICAM_CleanSerialRegisterString          "PICAM_CLEAN_SERIAL_REGISTER"
#define PICAM_CleanUntilTriggerString            "PICAM_CLEAN_UNTIL_TRIGGER"

//Sensor Temperature
#define PICAM_DisableCoolingFanString          "PICAM_DISABLE_COOLING_FAN"
#define PICAM_EnableSensorWindowHeaterString     "PICAM_ENABLE_SENSOR_WINDOW_HEATER"
#define PICAM_SensorTemperatureStatusString      "PICAM_SENSOR_TEMPERATURE_STATUS"

//Display Aids
#define PICAM_EnableROIMinXInputString         "PICAM_ENABLE_ROI_MINX_INPUT"
#define PICAM_EnableROISizeXInputString         "PICAM_ENABLE_ROI_SIZEX_INPUT"
#define PICAM_EnableROIMinYInputString         "PICAM_ENABLE_ROI_MINY_INPUT"
#define PICAM_EnableROISizeYInputString         "PICAM_ENABLE_ROI_SIZEY_INPUT"

// Camera Parameter Exists
#define PICAM_ExposureTimeExistsString           "PICAM_EXPOSURE_TIME_EX"
#define PICAM_ShutterClosingDelayExistsString    "PICAM_SHUTTER_CLOSING_DELAY_EX"
#define PICAM_ShutterDelayResolutionExistsString "PICAM_SHUTTER_DELAY_RESOLUTION_EX"
#define PICAM_ShutterOpeningDelayExistsString    "PICAM_SHUTTER_OPEN_DELAY_EX"
#define PICAM_ShutterTimingModeExistsString      "PICAM_SHUTTER_TIMING_MODE_EX"
#define PICAM_BracketGatingExistsString          "PICAM_BRACKET_GATING_EX"
#define PICAM_CustomModulationSequenceExistsString      "PICAM_CUSTOM_MODULATION_SEQUENCE_EX"
#define PICAM_DifEndingGateExistsString                 "PICAM_DIF_END_GATE_EX"
#define PICAM_DifStartingGateExistsString               "PICAM_DIF_START_GATE_EX"
#define PICAM_EMIccdGainExistsString                    "PICAM_EMI_CCD_GAIN_EX"
#define PICAM_EMIccdGainControlModeExistsString         "PICAM_EMI_CCD_GAIN_CTL_MODE_EX"
#define PICAM_EnableIntensifierExistsString             "PICAM_ENABLE_INTENSIFIER_EX"
#define PICAM_EnableModulationExistsString              "PICAM_ENABLE_MODULATION_EX"
#define PICAM_GatingModeExistsString                    "PICAM_GATING_MODE_EX"
#define PICAM_GatingSpeedExistsString                   "PICAM_GATING_SPEED_EX"
#define PICAM_IntensifierDiameterExistsString           "PICAM_INTENSIFIER_DIAM_EX"
#define PICAM_IntensifierGainExistsString               "PICAM_INTENSIFIER_GAIN_EX"
#define PICAM_IntensifierOptionsExistsString            "PICAM_INTENSIFIER_OPTIONS_EX"
#define PICAM_IntensifierStatusExistsString             "PICAM_INTENSIFIER_STATUS_EX"
#define PICAM_ModulationDurationExistsString            "PICAM_MODULATION_DURATION_EX"
#define PICAM_ModulationFrequencyExistsString           "PICAM_MODULATION_FREQUENCY_EX"
#define PICAM_PhosphorDecayDelayExistsString            "PICAM_PHOSFOR_DECAY_DELAY_EX"
#define PICAM_PhosphorDecayDelayResolutionExistsString  "PICAM_PHOSFOR_DELAY_DECAY_RES_EX"
#define PICAM_PhosphorTypeExistsString                  "PICAM_PHOSFOR_TYPE_EX"
#define PICAM_PhotocathodeSensitivityExistsString       "PICAM_PHOTOCATHODE_SENSITIVITY_EX"
#define PICAM_RepetitiveGateExistsString                "PICAM_REPETITIVE_GATE_EX"
#define PICAM_RepetitiveModulationPhaseExistsString     "PICAM_REPETITIVE_MODULATION_EX"
#define PICAM_SequentialStartingModulationPhaseExistsString "PICAM_SEQ_STARTING_MODULATION_PHASE_EX"
#define PICAM_SequentialEndingModulationPhaseExistsString   "PICAM_SEQ_END_MODULATION_PHASE_EX"
#define PICAM_SequentialEndingGateExistsString          "PICAM_SEQ_END_GATE_EX"
#define PICAM_SequentialGateStepCountExistsString       "PICAM_SEQ_GATE_STEP_COUNT_EX"
#define PICAM_SequentialGateStepIterationsExistsString  "PICAM_SEQ_GATE_STEP_ITERATIONS_EX"
#define PICAM_SequentialStartingGateExistsString        "PICAM_SEQ_START_GATE_EX"
#define PICAM_AdcAnalogGainExistsString                 "PICAM_ADC_ANALOG_GAIN_EX"
#define PICAM_AdcBitDepthExistsString                   "PICAM_ADC_BIT_DEPTH_EX"
#define PICAM_AdcEMGainExistsString                     "PICAM_ADC_EM_GAIN_EX"
#define PICAM_AdcQualityExistsString                    "PICAM_ADC_QUALITY_EX"
#define PICAM_AdcSpeedExistsString                      "PICAM_ADC_SPEED_EX"
#define PICAM_CorrectPixelBiasExistsString              "PICAM_CORRECT_PIXEL_BIAS_EX"
#define PICAM_AuxOutputExistsString                     "PICAM_AUX_OUTPUT_EX"
#define PICAM_EnableModulationOutputSignalExistsString          "PICAM_ENABLE_MODULATION_OUT_SIGNAL_EX"
#define PICAM_ModulationOutputSignalFrequencyExistsString "PICAM_MODULATION_OUT_SIGNAL_FREQ_EX"
#define PICAM_ModulationOutputSignalAmplitudeExistsString "PICAM_MODULATION_OUT_SIGNAL_AMPL_EX"
#define PICAM_EnableSyncMasterExistsString              "PICAM_SYNC_MASTER_EX"
#define PICAM_InvertOutputSignalExistsString            "PICAM_INVERT_OUTPUT_SIGNAL_EX"
#define PICAM_OutputSignalExistsString                  "PICAM_OUTPUT_SIGNAL_EX"
#define PICAM_SyncMaster2DelayExistsString              "PICAM_SYNC_MASTER2_DELAY_EX"
#define PICAM_TriggerCouplingExistsString               "PICAM_TRIGGER_COUPLING_EX"
#define PICAM_TriggerDeterminationExistsString          "PICAM_TRIGGER_DETERMINATION_EX"
#define PICAM_TriggerFrequencyExistsString              "PICAM_TRIGGER_FREQUENCY_EX"
#define PICAM_TriggerResponseExistsString               "PICAM_TRIGGER_RESPONSE_EX"
#define PICAM_TriggerSourceExistsString                 "PICAM_TRIGGER_SOURCE_EX"
#define PICAM_TriggerTerminationExistsString            "PICAM_TRIGGER_TERMINATION_EX"
#define PICAM_TriggerThresholdExistsString              "PICAM_TRIGGER_THRESHOLD_EX"
#define PICAM_AccumulationsExistsString                 "PICAM_ACCUMULATIONS_EX"
#define PICAM_EnableNondestructiveReadoutExistsString   "PICAM_ENABLE_NONDESTRUCT_READOUT_EX"
#define PICAM_KineticsWindowHeightExistsString          "PICAM_KINETICS_WINDOW_HEIGHT_EX"
#define PICAM_NondestructiveReadoutPeriodExistsString   "PICAM_NONDESTRUCT_READOUT_PERIOD_EX"
#define PICAM_ReadoutControlModeExistsString            "PICAM_READOUT_CONTROL_MODE_EX"
#define PICAM_ReadoutOrientationExistsString            "PICAM_READOUNT_ORIENTATION_EX"
#define PICAM_ReadoutPortCountExistsString              "PICAM_READOUT_PORT_COUNT_EX"
#define PICAM_ReadoutTimeCalculationExistsString        "PICAM_READOUT_TIME_CALC_EX"
#define PICAM_VerticalShiftRateExistsString             "PICAM_VERTICAL_SHIFT_RATE_EX"
#define PICAM_DisableDataFormattingExistsString         "PICAM_DISABLE_DATA_FORMATTING_EX"
#define PICAM_ExactReadoutCountMaximumExistsString      "PICAM_EXACT_READOUT_COUNT_MAX_EX"
#define PICAM_FrameRateCalculationExistsString          "PICAM_FRAME_RATE_CALC_EX"
#define PICAM_FrameSizeExistsString                     "PICAM_FRAME_SIZE_EX"
#define PICAM_FramesPerReadoutExistsString              "PICAM_FRAMES_PER_READOUT_EX"
#define PICAM_FrameStrideExistsString                   "PICAM_FRAME_STRIDE_EX"
#define PICAM_FrameTrackingBitDepthExistsString         "PICAM_FRAME_TRK_BIT_DEPTH_EX"
#define PICAM_GateTrackingExistsString                  "PICAM_GATE_TRACKING_EX"
#define PICAM_GateTrackingBitDepthExistsString          "PICAM_FRAME_TRACKING_BIT_DEPTH_EX"
#define PICAM_ModulationTrackingExistsString            "PICAM_MODULATION_TRACKING_EX"
#define PICAM_ModulationTrackingBitDepthExistsString    "PICAM_MODULATION_TRACKING_BIT_DEPTH_EX"
#define PICAM_NormalizeOrientationExistsString          "PICAM_NORMALIZE_ORIENTATION_EX"
#define PICAM_OnlineReadoutRateCalculationExistsString  "PICAM_ONLINE_READOUT_RATE_CALC_EX"
#define PICAM_OrientationExistsString                   "PICAM_ORIENTATION_EX"
#define PICAM_PhotonDetectionModeExistsString           "PICAM_PHOTON_DETECTION_MODE_EX"
#define PICAM_PhotonDetectionThresholdExistsString      "PICAM_PHOTON_DETECT_THRESHOLD_EX"
#define PICAM_PixelBitDepthExistsString                 "PICAM_PIXEL_BIT_DEPTH_EX"
#define PICAM_PixelFormatExistsString                   "PICAM_PIXEL_FORMAT_EX"
#define PICAM_ReadoutCountExistsString                  "PICAM_READOUT_COUNT_EX"
#define PICAM_ReadoutRateCalculationExistsString   "PICAM_READOUT_RATE_CALC_EX"
#define PICAM_ReadoutStrideExistsString                 "PICAM_READOUT_STRIDE_EX"
#define PICAM_RoisExistsString                          "PICAM_ROIS_EX"
#define PICAM_TimeStampBitDepthExistsString             "PICAM_TIME_STAMP_BIT_DEPTH_EX"
#define PICAM_TimeStampResolutionExistsString           "PICAM_TIME_STAMP_RESOLUTION_EX"
#define PICAM_TimeStampsExistsString                    "PICAM_TIME_STAMPS_EX"
#define PICAM_TrackFramesExistsString                   "PICAM_TRACK_FRAMES_EX"
#define PICAM_CcdCharacteristicsExistsString            "PICAM_CCD_CHARACTERISTICS_EX"
#define PICAM_PixelGapHeightExistsString                "PICAM_PIXEL_GAP_HEIGHT_EX"
#define PICAM_PixelGapWidthExistsString                 "PICAM_PIXEL_GAP_WIDTH_EX"
#define PICAM_PixelHeightExistsString                   "PICAM_PIXEL_HEIGHT_EX"
#define PICAM_PixelWidthExistsString                    "PICAM_PIXEL_WIDTH_EX"
#define PICAM_SensorActiveBottomMarginExistsString      "PICAM_SENSOR_ACTIVE_BOTTOM_MARGIN_EX"
#define PICAM_SensorActiveHeightExistsString            "PICAM_SENSOR_ACTIVE_HEIGHT_EX"
#define PICAM_SensorActiveLeftMarginExistsString        "PICAM_SENSOR_ACTIVE_LEFT_MARGIN_EX"
#define PICAM_SensorActiveRightMarginExistsString       "PICAM_SENSOR_ACTIVE_RIGHT_MARGIN_EX"
#define PICAM_SensorActiveTopMarginExistsString         "PICAM_SENSOR_ACTIVE_TOP_MARGIN_EX"
#define PICAM_SensorActiveWidthExistsString             "PICAM_SENSOR_ACTIVE_WIDTH_EX"
#define PICAM_SensorMaskedBottomMarginExistsString      "PICAM_SENSOR_MASK_BOTTOM_MARGIN_EX"
#define PICAM_SensorMaskedHeightExistsString            "PICAM_SENSOR_MASK_HEIGHT_EX"
#define PICAM_SensorMaskedTopMarginExistsString         "PICAM_SENSOR_MASK_TOP_MARGIN_EX"
#define PICAM_SensorSecondaryActiveHeightExistsString   "PICAM_SENSOR_SECONDARY_ACTIVE_HEIGHT_EX"
#define PICAM_SensorSecondaryMaskedHeightExistsString   "PICAM_SENSOR_SECONDARY_MASK_HEIGHT_EX"
#define PICAM_SensorTypeExistsString                   "PICAM_SENSOR_TYPE_EX"
#define PICAM_ActiveBottomMarginExistsString            "PICAM_ACTIVE_BOTTOM_MARGIN_EX"
#define PICAM_ActiveHeightExistsString                  "PICAM_ACTIVE_HEIGHT_EX"
#define PICAM_ActiveLeftMarginExistsString              "PICAM_ACTIVE_LEFT_MARGIN_EX"
#define PICAM_ActiveRightMarginExistsString             "PICAM_ACTIVE_RIGHT_MARGIN_EX"
#define PICAM_ActiveTopMarginExistsString               "PICAM_ACTIVE_TOP_MARGIN_EX"
#define PICAM_ActiveWidthExistsString                   "PICAM_ACTIVE_WIDTH_EX"
#define PICAM_MaskedBottomMarginExistsString            "PICAM_MASK_BOTTOM_MARGIN_EX"
#define PICAM_MaskedHeightExistsString                  "PICAM_MASK_HEIGHT_EX"
#define PICAM_MaskedTopMarginExistsString               "PICAM_MASK_TOP_MARGIN_EX"
#define PICAM_SecondaryActiveHeightExistsString         "PICAM_SECONDARY_ACTIVE_HEIGHT_EX"
#define PICAM_SecondaryMaskedHeightExistsString         "PICAM_SECONDARY_MASK_HEIGHT_EX"
#define PICAM_CleanBeforeExposureExistsString           "PICAM_CLEAN_BEFORE_EXPOSURE_EX"
#define PICAM_CleanCycleCountExistsString               "PICAM_CLEAN_CYCLE_COUNT_EX"
#define PICAM_CleanCycleHeightExistsString              "PICAM_CLEAN_CYCLE_HEIGHT_EX"
#define PICAM_CleanSectionFinalHeightExistsString       "PICAM_CLEAN_SECTION_FINAL_HEIGHT_EX"
#define PICAM_CleanSectionFinalHeightCountExistsString  "PICAM_CLEAN_SECTION_FINAL_COUNT_EX"
#define PICAM_CleanSerialRegisterExistsString           "PICAM_CLEAN_SERIAL_REGISTER_EX"
#define PICAM_CleanUntilTriggerExistsString             "PICAM_CLEAN_UNTIL_TRIG_EX"
#define PICAM_DisableCoolingFanExistsString             "PICAM_DISABLE_COOLING_FAN_EX"
#define PICAM_EnableSensorWindowHeaterExistsString      "PICAM_ENABLE_WINDOW_SENSOR_HEATER_EX"
#define PICAM_SensorTemperatureReadingExistsString      "PICAM_SENSOR_TEMPERATURE_READING_EX"
#define PICAM_SensorTemperatureSetPointExistsString     "PICAM_SENSOR_TEMPERATURE_SETPOINT_EX"
#define PICAM_SensorTemperatureStatusExistsString       "PICAM_SENSOR_TEMPERATURE_STATUS_EX"
// Camera Parameter Relevance
#define PICAM_ExposureTimeRelString           "PICAM_EXPOSURE_TIME_PR"
#define PICAM_ShutterClosingDelayRelString    "PICAM_SHUTTER_CLOSING_DELAY_PR"       
#define PICAM_ShutterDelayResolutionRelString "PICAM_SHUTTER_DELAY_RESOLUTION_PR"     
#define PICAM_ShutterOpeningDelayRelString    "PICAM_SHUTTER_OPEN_DELAY_PR" 
#define PICAM_ShutterTimingModeRelString      "PICAM_SHUTTER_TIMING_MODE_PR"
#define PICAM_BracketGatingRelString          "PICAM_BRACKET_GATING_PR"
#define PICAM_CustomModulationSequenceRelString      "PICAM_CUSTOM_MODULATION_SEQUENCE_PR"  
#define PICAM_DifEndingGateRelString                 "PICAM_DIF_END_GATE_PR"
#define PICAM_DifStartingGateRelString               "PICAM_DIF_START_GATE_PR"
#define PICAM_EMIccdGainRelString                    "PICAM_EMI_CCD_GAIN_PR"
#define PICAM_EMIccdGainControlModeRelString         "PICAM_EMI_CCD_GAIN_CTL_MODE_PR"
#define PICAM_EnableIntensifierRelString             "PICAM_ENABLE_INTENSIFIER_PR"  
#define PICAM_EnableModulationRelString              "PICAM_ENABLE_MODULATION_PR"
#define PICAM_GatingModeRelString                    "PICAM_GATING_MODE_PR"
#define PICAM_GatingSpeedRelString                   "PICAM_GATING_SPEED_PR"
#define PICAM_IntensifierDiameterRelString           "PICAM_INTENSIFIER_DIAM_PR"
#define PICAM_IntensifierGainRelString               "PICAM_INTENSIFIER_GAIN_PR"
#define PICAM_IntensifierOptionsRelString            "PICAM_INTENSIFIER_OPTIONS_PR"
#define PICAM_IntensifierStatusRelString             "PICAM_INTENSIFIER_STATUS_PR"
#define PICAM_ModulationDurationRelString            "PICAM_MODULATION_DURATION_PR"
#define PICAM_ModulationFrequencyRelString           "PICAM_MODULATION_FREQUENCY_PR"
#define PICAM_PhosphorDecayDelayRelString            "PICAM_PHOSFOR_DECAY_DELAY_PR"
#define PICAM_PhosphorDecayDelayResolutionRelString  "PICAM_PHOSFOR_DELAY_DECAY_RES_PR"  
#define PICAM_PhosphorTypeRelString                  "PICAM_PHOSFOR_TYPE_PR"
#define PICAM_PhotocathodeSensitivityRelString       "PICAM_PHOTOCATHODE_SENSITIVITY_PR"
#define PICAM_RepetitiveGateRelString                "PICAM_REPETITIVE_GATE_PR"
#define PICAM_RepetitiveModulationPhaseRelString     "PICAM_REPETITIVE_MODULATION_PR" 
#define PICAM_SequentialStartingModulationPhaseRelString "PICAM_SEQ_STARTING_MODULATION_PHASE_PR"
#define PICAM_SequentialEndingModulationPhaseRelString   "PICAM_SEQ_END_MODULATION_PHASE_PR"
#define PICAM_SequentialEndingGateRelString          "PICAM_SEQ_END_GATE_PR"
#define PICAM_SequentialGateStepCountRelString       "PICAM_SEQ_GATE_STEP_COUNT_PR"
#define PICAM_SequentialGateStepIterationsRelString  "PICAM_SEQ_GATE_STEP_ITERATIONS_PR"    
#define PICAM_SequentialStartingGateRelString        "PICAM_SEQ_START_GATE_PR" 
#define PICAM_AdcAnalogGainRelString                 "PICAM_ADC_ANALOG_GAIN_PR"
#define PICAM_AdcBitDepthRelString                   "PICAM_ADC_BIT_DEPTH_PR"
#define PICAM_AdcEMGainRelString                     "PICAM_ADC_EM_GAIN_PR"
#define PICAM_AdcQualityRelString                    "PICAM_ADC_QUALITY_PR"
#define PICAM_AdcSpeedRelString                      "PICAM_ADC_SPEED_PR"
#define PICAM_CorrectPixelBiasRelString              "PICAM_CORRECT_PIXEL_BIAS_PR" 
#define PICAM_AuxOutputRelString                     "PICAM_AUX_OUTPUT_PR"
#define PICAM_EnableModulationOutputSignalRelString          "PICAM_ENABLE_MODULATION_OUT_SIGNAL_PR"
#define PICAM_ModulationOutputSignalFrequencyRelString "PICAM_MODULATION_OUT_SIGNAL_FREQ_PR"
#define PICAM_ModulationOutputSignalAmplitudeRelString "PICAM_MODULATION_OUT_SIGNAL_AMPL_PR"
#define PICAM_EnableSyncMasterRelString              "PICAM_SYNC_MASTER_PR"
#define PICAM_InvertOutputSignalRelString            "PICAM_INVERT_OUTPUT_SIGNAL_PR" 
#define PICAM_OutputSignalRelString                  "PICAM_OUTPUT_SIGNAL_PR"
#define PICAM_SyncMaster2DelayRelString              "PICAM_SYNC_MASTER2_DELAY_PR"    
#define PICAM_TriggerCouplingRelString               "PICAM_TRIGGER_COUPLING_PR"
#define PICAM_TriggerDeterminationRelString          "PICAM_TRIGGER_DETERMINATION_PR" 
#define PICAM_TriggerFrequencyRelString              "PICAM_TRIGGER_FREQUENCY_PR"
#define PICAM_TriggerResponseRelString               "PICAM_TRIGGER_RESPONSE_PR"
#define PICAM_TriggerSourceRelString                 "PICAM_TRIGGER_SOURCE_PR"
#define PICAM_TriggerTerminationRelString            "PICAM_TRIGGER_TERMINATION_PR"
#define PICAM_TriggerThresholdRelString              "PICAM_TRIGGER_THRESHOLD_PR"
#define PICAM_AccumulationsRelString                 "PICAM_ACCUMULATIONS_PR"
#define PICAM_EnableNondestructiveReadoutRelString   "PICAM_ENABLE_NONDESTRUCT_READOUT_PR"   
#define PICAM_KineticsWindowHeightRelString          "PICAM_KINETICS_WINDOW_HEIGHT_PR"
#define PICAM_NondestructiveReadoutPeriodRelString   "PICAM_NONDESTRUCT_READOUT_PERIOD_PR"
#define PICAM_ReadoutControlModeRelString            "PICAM_READOUT_CONTROL_MODE_PR"
#define PICAM_ReadoutOrientationRelString            "PICAM_READOUNT_ORIENTATION_PR"
#define PICAM_ReadoutPortCountRelString              "PICAM_READOUT_PORT_COUNT_PR"
#define PICAM_ReadoutTimeCalculationRelString        "PICAM_READOUT_TIME_CALC_PR"
#define PICAM_VerticalShiftRateRelString             "PICAM_VERTICAL_SHIFT_RATE_PR"
#define PICAM_DisableDataFormattingRelString         "PICAM_DISABLE_DATA_FORMATTING_PR"
#define PICAM_ExactReadoutCountMaximumRelString      "PICAM_EXACT_READOUT_COUNT_MAX_PR"
#define PICAM_FrameRateCalculationRelString          "PICAM_FRAME_RATE_CALC_PR"
#define PICAM_FrameSizeRelString                     "PICAM_FRAME_SIZE_PR"
#define PICAM_FramesPerReadoutRelString              "PICAM_FRAMES_PER_READOUT_PR"
#define PICAM_FrameStrideRelString                   "PICAM_FRAME_STRIDE_PR"
#define PICAM_FrameTrackingBitDepthRelString         "PICAM_FRAME_TRK_BIT_DEPTH_PR"
#define PICAM_GateTrackingRelString                  "PICAM_GATE_TRACKING_PR"
#define PICAM_GateTrackingBitDepthRelString          "PICAM_FRAME_TRACKING_BIT_DEPTH_PR"
#define PICAM_ModulationTrackingRelString            "PICAM_MODULATION_TRACKING_PR"
#define PICAM_ModulationTrackingBitDepthRelString    "PICAM_MODULATION_TRACKING_BIT_DEPTH_PR"   
#define PICAM_NormalizeOrientationRelString          "PICAM_NORMALIZE_ORIENTATION_PR"
#define PICAM_OnlineReadoutRateCalculationRelString  "PICAM_ONLINE_READOUT_RATE_CALC_PR"
#define PICAM_OrientationRelString                   "PICAM_ORIENTATION_PR"
#define PICAM_PhotonDetectionModeRelString           "PICAM_PHOTON_DETECTION_MODE_PR" 
#define PICAM_PhotonDetectionThresholdRelString      "PICAM_PHOTON_DETECT_THRESHOLD_PR"
#define PICAM_PixelBitDepthRelString                 "PICAM_PIXEL_BIT_DEPTH_PR"
#define PICAM_PixelFormatRelString                   "PICAM_PIXEL_FORMAT_PR"
#define PICAM_ReadoutCountRelString                  "PICAM_READOUT_COUNT_PR"
#define PICAM_ReadoutRateCalculationRelString   "PICAM_READOUT_RATE_CALC_PR"  
#define PICAM_ReadoutStrideRelString                 "PICAM_READOUT_STRIDE_PR"
#define PICAM_RoisRelString                          "PICAM_ROIS_PR"
#define PICAM_TimeStampBitDepthRelString             "PICAM_TIME_STAMP_BIT_DEPTH_PR"
#define PICAM_TimeStampResolutionRelString           "PICAM_TIME_STAMP_RESOLUTION_PR"  
#define PICAM_TimeStampsRelString                    "PICAM_TIME_STAMPS_PR"
#define PICAM_TrackFramesRelString                   "PICAM_TRACK_FRAMES_PR"   
#define PICAM_CcdCharacteristicsRelString            "PICAM_CCD_CHARACTERISTICS_PR"
#define PICAM_PixelGapHeightRelString                "PICAM_PIXEL_GAP_HEIGHT_PR"
#define PICAM_PixelGapWidthRelString                 "PICAM_PIXEL_GAP_WIDTH_PR"
#define PICAM_PixelHeightRelString                   "PICAM_PIXEL_HEIGHT_PR"
#define PICAM_PixelWidthRelString                    "PICAM_PIXEL_WIDTH_PR"
#define PICAM_SensorActiveBottomMarginRelString      "PICAM_SENSOR_ACTIVE_BOTTOM_MARGIN_PR"
#define PICAM_SensorActiveHeightRelString            "PICAM_SENSOR_ACTIVE_HEIGHT_PR"
#define PICAM_SensorActiveLeftMarginRelString        "PICAM_SENSOR_ACTIVE_LEFT_MARGIN_PR"
#define PICAM_SensorActiveRightMarginRelString       "PICAM_SENSOR_ACTIVE_RIGHT_MARGIN_PR"
#define PICAM_SensorActiveTopMarginRelString         "PICAM_SENSOR_ACTIVE_TOP_MARGIN_PR"
#define PICAM_SensorActiveWidthRelString             "PICAM_SENSOR_ACTIVE_WIDTH_PR"
#define PICAM_SensorMaskedBottomMarginRelString      "PICAM_SENSOR_MASK_BOTTOM_MARGIN_PR"
#define PICAM_SensorMaskedHeightRelString            "PICAM_SENSOR_MASK_HEIGHT_PR"
#define PICAM_SensorMaskedTopMarginRelString         "PICAM_SENSOR_MASK_TOP_MARGIN_PR"
#define PICAM_SensorSecondaryActiveHeightRelString   "PICAM_SENSOR_SECONDARY_ACTIVE_HEIGHT_PR"  
#define PICAM_SensorSecondaryMaskedHeightRelString   "PICAM_SENSOR_SECONDARY_MASK_HEIGHT_PR"   
#define PICAM_SensorTypeRelString                   "PICAM_SENSOR_TYPE_PR"
#define PICAM_ActiveBottomMarginRelString            "PICAM_ACTIVE_BOTTOM_MARGIN_PR"   
#define PICAM_ActiveHeightRelString                  "PICAM_ACTIVE_HEIGHT_PR"
#define PICAM_ActiveLeftMarginRelString              "PICAM_ACTIVE_LEFT_MARGIN_PR"
#define PICAM_ActiveRightMarginRelString             "PICAM_ACTIVE_RIGHT_MARGIN_PR"
#define PICAM_ActiveTopMarginRelString               "PICAM_ACTIVE_TOP_MARGIN_PR"
#define PICAM_ActiveWidthRelString                   "PICAM_ACTIVE_WIDTH_PR"
#define PICAM_MaskedBottomMarginRelString            "PICAM_MASK_BOTTOM_MARGIN_PR"  
#define PICAM_MaskedHeightRelString                  "PICAM_MASK_HEIGHT_PR"
#define PICAM_MaskedTopMarginRelString               "PICAM_MASK_TOP_MARGIN_PR"
#define PICAM_SecondaryActiveHeightRelString         "PICAM_SECONDARY_ACTIVE_HEIGHT_PR"
#define PICAM_SecondaryMaskedHeightRelString         "PICAM_SECONDARY_MASK_HEIGHT_PR"
#define PICAM_CleanBeforeExposureRelString           "PICAM_CLEAN_BEFORE_EXPOSURE_PR"
#define PICAM_CleanCycleCountRelString               "PICAM_CLEAN_CYCLE_COUNT_PR"
#define PICAM_CleanCycleHeightRelString              "PICAM_CLEAN_CYCLE_HEIGHT_PR"
#define PICAM_CleanSectionFinalHeightRelString       "PICAM_CLEAN_SECTION_FINAL_HEIGHT_PR"      
#define PICAM_CleanSectionFinalHeightCountRelString  "PICAM_CLEAN_SECTION_FINAL_COUNT_PR"      
#define PICAM_CleanSerialRegisterRelString           "PICAM_CLEAN_SERIAL_REGISTER_PR"
#define PICAM_CleanUntilTriggerRelString             "PICAM_CLEAN_UNTIL_TRIG_PR"
#define PICAM_DisableCoolingFanRelString             "PICAM_DISABLE_COOLING_FAN_PR"
#define PICAM_EnableSensorWindowHeaterRelString      "PICAM_ENABLE_WINDOW_SENSOR_HEATER_PR"
#define PICAM_SensorTemperatureReadingRelString      "PICAM_SENSOR_TEMPERATURE_READING_PR"       
#define PICAM_SensorTemperatureSetPointRelString     "PICAM_SENSOR_TEMPERATURE_SETPOINT_PR"       
#define PICAM_SensorTemperatureStatusRelString       "PICAM_SENSOR_TEMPERATURE_STATUS_PR"      

#define NUM_PICAM_PARAMS ((int)(&PICAM_LAST_PARAM - &PICAM_FIRST_PARAM + 1))
//_____________________________________________________________________________

#endif
