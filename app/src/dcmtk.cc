// Copyright 2016 Tamas Palagyi

#include "dcmtk.hh"

// Logging
#include <easylogging++.h>

// DCMTK
#include <dcmtk/dcmdata/dcfilefo.h>
#include <dcmtk/dcmdata/dctk.h>
#include <dcmtk/dcmnet/assoc.h>
#include <dcmtk/dcmnet/dimse.h>
#include <dcmtk/dcmnet/diutil.h>
#include <dcmtk/dcmdata/dcdeftag.h>
#include "dcmtk/dcmjpls/djencode.h"   /* for class DJLSEncoderRegistration */
#include "dcmtk/dcmjpls/djdecode.h"   /* for class DJLSDecoderRegistration */
#include "dcmtk/dcmjpls/djrparam.h"   /* for class DJLSRepresentationParameter */

#include "dcmtk/dcmjpeg/djrplol.h"
#include "dcmtk/dcmjpeg/djrploss.h"
#include "dcmtk/dcmjpeg/djencode.h"   /* for dcmjpeg encoders */
#include "dcmtk/dcmjpeg/djdecode.h"   /* for dcmjpeg decoders */

#include "dcmtk/dcmdata/dcrledrg.h"  /* for RLE decoder */
#include "dcmtk/dcmdata/dcrleerg.h"  /* for RLE encoder */
#include "dcmtk/dcmdata/dctypes.h"   /* for DcmData.print flags */

#include "dcmtk/dcmnet/dimse.h"

#include "dcmtk/dcmdata/dctk.h" 
#include "dcmtk/dcmdata/dcpxitem.h"
 
#include "fmjpeg2k/djdecode.h" /* for J2K decoder */
#include "fmjpeg2k/djencode.h" /* for J2K encoder */

#include <openssl/sha.h>

#include "imagestore.hh"

//------------------------------------------------------------------------------

const int test = 12;

// All supported transfer synaxes by the C-STORE SCU
const char* _transfer_syntaxes[] = {
  // Uncompressed
  UID_LittleEndianExplicitTransferSyntax,
  UID_BigEndianExplicitTransferSyntax,
  UID_LittleEndianImplicitTransferSyntax,
  // Lossless compressed
  UID_JPEGLSLosslessTransferSyntax,
  UID_JPEGProcess14SV1TransferSyntax,
  UID_RLELosslessTransferSyntax,
  UID_JPEG2000LosslessOnlyTransferSyntax,
  // Lossy comressed
  UID_JPEG2000TransferSyntax,
  UID_JPEGProcess1TransferSyntax,
  UID_JPEGProcess2_4TransferSyntax,
  UID_JPEGLSLossyTransferSyntax};

const int length_txs = DIM_OF(_transfer_syntaxes);

//------------------------------------------------------------------------------
// Helper functions
//------------------------------------------------------------------------------

void chk_cond(const char* file, int line, OFCondition cond) {
  if (cond.bad()) {
    throw dicom_exception(cond.text(), cond.code(), file, line);
  }
}

void chk_cond(const char* file, int line, OFBool res) {
  if (!res) {
    throw dicom_exception("Returned False.", -1, file, line);
  }
}

void info_association_rq(T_ASC_Association* assoc) {
  LOG(INFO) << "Association Received ("
            << assoc->params->DULparams.callingPresentationAddress
            << ":" << assoc->params->DULparams.callingAPTitle << " -> "
            << assoc->params->DULparams.calledAPTitle << ")";

  OFString temp_str;
  LOG(INFO) << "Parameters:" << OFendl << ASC_dumpParameters(temp_str, assoc->params, ASC_ASSOC_RQ);
}

void info_association_ac(T_ASC_Association* assoc) {
  LOG(INFO) << "Association Acknowledged (Max Send PDV: " << assoc->sendPDVLength << ")";
  if (ASC_countAcceptedPresentationContexts(assoc->params) == 0) {
    LOG(INFO) << "    (but no valid presentation contexts)";
  }

  OFString temp_str;
  LOG(INFO) << ASC_dumpParameters(temp_str, assoc->params, ASC_ASSOC_AC);
}

//------------------------------------------------------------------------------
// Class methods
//------------------------------------------------------------------------------

// C-MOVE SCP
DicomDcmtk::DicomDcmtk(int port,
                       const std::string& aet,
                       const ImageStore* imagestore,
                       const AetMap* rhosts,
                       bool multi,
                       bool need_response)
  : Dicom(port, aet, imagestore, rhosts, multi, need_response),
    network_(0),
    assoc_(0) {

  LOG(INFO) << "Initialize network as NET_ACCEPTORREQUESTOR on port " << dicom_port_;
  //  DUL_requestForkOnTransportConnectionReceipt(0, 0);
  CHK(ASC_initializeNetwork(NET_ACCEPTORREQUESTOR, dicom_port_, timeout_, &network_));
}

// C-MOVE SCU
DicomDcmtk::DicomDcmtk(const std::string& aet,
                       const std::string& host,
                       int port,
                       bool need_response)
  : Dicom(aet, host, port, need_response),
    network_(0),
    assoc_(0) {

  LOG(INFO) << "Initialize network as NET_REQUESTOR";
  //  DUL_requestForkOnTransportConnectionReceipt(0, 0);
  CHK(ASC_initializeNetwork(NET_REQUESTOR, 0, timeout_, &network_));
}

//------------------------------------------------------------------------------

void convert_to_supported_transfer_syntaxes(DcmDataset* original, DatasetMap* dataset_map, const Xfers& xfers) {
  Xfers xfers2;
  if (xfers.empty()) {
    LOG(INFO) << "Adding default list of Xfers.";
    for (auto transfer_syntax : _transfer_syntaxes) {
      xfers2.insert(transfer_syntax);
    }
  }

  for (auto xfer : xfers) {
    DcmXfer xferobj(xfer.c_str());

    std::string result_str;
    std::string reason = "";

    DcmDataset* dataset_copy = new DcmDataset(*original);
    OFCondition res = dataset_copy->chooseRepresentation(xferobj.getXfer(), 0);
    (*dataset_map)[xfer] = dataset_copy;

    if (res.good()) {
      result_str = "conversion OK";
    } else {
      result_str = "conversion ERROR: ";
      reason = res.text();
    }

    LOG(INFO) << xferobj.getXferName()
              << " (" <<  xferobj.getXferID() << ") "
              << result_str << reason;
  }
}

//------------------------------------------------------------------------------

void convert_buf_to_hex(const unsigned char* data, const size_t length) {
  const size_t kSize = 2 * length + 1;
  char obuffer[kSize];
  obuffer[kSize - 1] = 0;

  for (size_t j = 0; j < length; j++)
    snprintf(&obuffer[2*j], kSize, "%02X", data[j]);

  LOG(INFO) << "MD> " << static_cast<char*>(obuffer);
}

void calculate_checksum(const unsigned char* buffer, size_t length) {
  unsigned char md[SHA_DIGEST_LENGTH];
  SHA1(buffer, length, md);

  convert_buf_to_hex(md, SHA_DIGEST_LENGTH);
}

void log_dataset_checksum(DcmDataset* dataset) {
  LOG(INFO) << "Dataset=" << dataset;
  dataset->print(std::cout,
                 // DCMTypes::PF_showTreeStructure |
                 DCMTypes::PF_useANSIEscapeCodes |
                 DCMTypes::PF_shortenLongTagValues);

  DcmStack stack;
  size_t i = 1;
  OFCondition status = dataset->nextObject(stack, OFTrue);
  while (status.good()) {
    DcmObject* dobject = stack.top();
    LOG(INFO) << i++ << ". " << dobject << " Type: " << typeid(*dobject).name();
    // dobject->print(std::cout);

    // Next element.
    status = dataset->nextObject(stack, OFTrue);
  }

  const Uint8* pixelData = 0;
  unsigned long num_elem = 0;
  CHK(dataset->findAndGetUint8Array(DCM_PixelData, pixelData, &num_elem));
  // LOG(INFO) << "PixelData=" << pixelData;
  LOG(INFO) << "num_elem=" << num_elem;

  calculate_checksum(pixelData, num_elem);
}

//------------------------------------------------------------------------------

void DicomDcmtk::load_image_file(const std::string& path, Image* image, const Xfers& xfers) {
  DcmFileFormat* fileformat = new DcmFileFormat();
  CHK(fileformat->loadFile(path.c_str()));
  CHK(fileformat->loadAllDataIntoMemory());

  DcmMetaInfo* metainfo = fileformat->getMetaInfo();
  DcmDataset* dataset = fileformat->getAndRemoveDataset();

  // log_dataset_checksum(dataset);

  // STUDY LEVEL
  OFString patient_id;
  OFString patient_name;
  OFString study_uid;
  OFString study_desc;

  // SERIES LEVEL
  OFString series_uid;
  OFString series_desc;

  // IMAGE LEVEL
  OFString sop_instance_uid;
  OFString sop_class_uid;

  OFString transfer_syntax_uid;
  // We must have all three to consider it into the datastructure
  if (dataset->findAndGetOFString(DCM_StudyInstanceUID, study_uid).bad() ||
      dataset->findAndGetOFString(DCM_SeriesInstanceUID, series_uid).bad() ||
      dataset->findAndGetOFString(DCM_SOPInstanceUID, sop_instance_uid).bad() ||
      metainfo->findAndGetOFString(DCM_TransferSyntaxUID, transfer_syntax_uid).bad()) {
    THROW_DICOM_EXCEPTION("Could not parse image.");
  }

  LOG(INFO) << "-----------------------------------------------------------------------";
  LOG(INFO) << "[" << study_uid << "][" << series_uid << "][" << sop_instance_uid << "]";

  // OPTIONALS
  dataset->findAndGetOFString(DCM_PatientID, patient_id);
  dataset->findAndGetOFString(DCM_PatientName, patient_name);
  dataset->findAndGetOFString(DCM_StudyDescription, study_desc);
  dataset->findAndGetOFString(DCM_SeriesDescription, series_desc);
  dataset->findAndGetOFString(DCM_SOPClassUID, sop_class_uid);

  convert_to_supported_transfer_syntaxes(dataset, &image->datasets_, xfers);

  E_TransferSyntax xferid = dataset->getOriginalXfer();
  DcmXfer xferobj(xferid);
  VLOG(1) << "Original Xfer: "
          << xferobj.getXferName()
          << " (" <<  xferobj.getXferID() << ")";

  // image->dataset_ = dataset;

  image->init(path,
              transfer_syntax_uid.c_str(),

              patient_id.c_str(),
              patient_name.c_str(),
              study_uid.c_str(),
              study_desc.c_str(),

              series_uid.c_str(),
              series_desc.c_str(),

              sop_instance_uid.c_str(),
              sop_class_uid.c_str());

  LOG(INFO) << "Supported xfers:";
  for (auto& kv : image->datasets_) {
    LOG(INFO) << kv.first;
  }
}

//------------------------------------------------------------------------------

void DicomDcmtk::init() {
  LOG(INFO) << "Not needed " << dicom_port_;
}

void DicomDcmtk::init_codec() {
  DJLSEncoderRegistration::cleanup();
  DJLSEncoderRegistration::registerCodecs();
  DJLSDecoderRegistration::cleanup();
  DJLSDecoderRegistration::registerCodecs();

  DJEncoderRegistration::cleanup();
  DJEncoderRegistration::registerCodecs(ECC_lossyYCbCr, EUC_never);
  DJDecoderRegistration::cleanup();
  DJDecoderRegistration::registerCodecs();

  DcmRLEEncoderRegistration::cleanup();
  DcmRLEEncoderRegistration::registerCodecs();
  DcmRLEDecoderRegistration::cleanup();
  DcmRLEDecoderRegistration::registerCodecs();

  FMJPEG2KEncoderRegistration::cleanup();
  FMJPEG2KEncoderRegistration::registerCodecs();
  FMJPEG2KDecoderRegistration::cleanup();
  FMJPEG2KDecoderRegistration::registerCodecs();
}

//------------------------------------------------------------------------------

void DicomDcmtk::waitfor_association() {
  LOG(INFO) << "Waiting for association...";
  CHK(ASC_associationWaiting(network_, timeout_));
  CHK(ASC_receiveAssociation(network_, &assoc_, ASC_DEFAULTMAXPDU));
  LOG(INFO) << "Incoming association request.";
}

//------------------------------------------------------------------------------

// FIXME: This could go to destructor if we split
// network and served association
void cleanup_f(T_ASC_Association* assoc) {
  ASC_dropAssociation(assoc);
  ASC_destroyAssociation(&assoc);
}

void DicomDcmtk::cleanup() {
  ASC_dropAssociation(assoc_);
  ASC_destroyAssociation(&assoc_);
}
//------------------------------------------------------------------------------

OFCondition negotiateAssociation(T_ASC_Association* assoc) {
  OFCondition cond;
  const char* transferSyntaxes[] = {UID_LittleEndianExplicitTransferSyntax,
                                    UID_BigEndianExplicitTransferSyntax,
                                    UID_LittleEndianImplicitTransferSyntax};

  const char* selectedNonStorageSyntaxes[] =
    {UID_VerificationSOPClass,
     UID_FINDPatientRootQueryRetrieveInformationModel,
     UID_MOVEPatientRootQueryRetrieveInformationModel,
     UID_GETPatientRootQueryRetrieveInformationModel,

     UID_RETIRED_FINDPatientStudyOnlyQueryRetrieveInformationModel,
     UID_RETIRED_MOVEPatientStudyOnlyQueryRetrieveInformationModel,
     UID_RETIRED_GETPatientStudyOnlyQueryRetrieveInformationModel,

     UID_FINDStudyRootQueryRetrieveInformationModel,
     UID_MOVEStudyRootQueryRetrieveInformationModel,
     UID_GETStudyRootQueryRetrieveInformationModel,
     UID_PrivateShutdownSOPClass};

  // accept any of the non-storage syntaxes
  if ((cond = ASC_acceptContextsWithPreferredTransferSyntaxes(assoc->params,
                                                              (const char**)selectedNonStorageSyntaxes,
                                                              DIM_OF(selectedNonStorageSyntaxes),
                                                              (const char**)transferSyntaxes,
                                                              DIM_OF(transferSyntaxes))).bad()) {
    return cond;
  }

  // accept storage syntaxes with proposed role
  for (int i = 0; i < ASC_countPresentationContexts(assoc->params); i++) {
    T_ASC_PresentationContext pc;
    ASC_getPresentationContext(assoc->params, i, &pc);
    if (dcmIsaStorageSOPClassUID(pc.abstractSyntax)) {
      // We are prepared to accept whatever role he proposes.
      // Normally we can be the SCP of the Storage Service Class.
      // When processing the C-GET operation we can be the SCU of the Storage Service Class.
      T_ASC_SC_ROLE role = pc.proposedRole;

      // Accept in the order "least wanted" to "most wanted" transfer
      // syntax.  Accepting a transfer syntax will override previously
      // accepted transfer syntaxes.
      for (int k = DIM_OF(transferSyntaxes) - 1; k >= 0; k--) {
        for (int j = 0; j < static_cast<int>(pc.transferSyntaxCount); j++) {
          // if the transfer syntax was proposed then we can accept it
          // appears in our supported list of transfer syntaxes
          if (strcmp(pc.proposedTransferSyntaxes[j], transferSyntaxes[k]) == 0) {
            if ((cond = ASC_acceptPresentationContext(assoc->params,
                                                      pc.presentationContextID,
                                                      transferSyntaxes[k],
                                                      role)).bad()) {
              return cond;
            }
          }
        }
      }
    }
  }

  return cond;
}

//------------------------------------------------------------------------------

struct CallbackData_SCP {
  T_ASC_Network* network_;
  T_ASC_Association* move_assoc_;
  T_ASC_Association* store_assoc_;

  const ImageStore* imagestore_;

  ImageStore::ImageMap* response_;
  ImageStore::ImageMap::const_iterator image_it_;
  ImageStore::ImageMap::const_iterator image_it_end_;

  ImageStore::SeriesMap* series_response_;
  ImageStore::SeriesMap::const_iterator series_it_;
  ImageStore::SeriesMap::const_iterator series_it_end_;

  std::string aet_;
  std::string host_;

  std::string calling_aet_;
  std::string calling_host_;

  std::string raet_;
  std::string rhost_;

  bool multi_;
  bool need_response_;

  DIC_US originatorid_;
};

struct CallbackData_SCU {
  el::base::PerformanceTracker* timer_;
  std::string xfer_;
  int dicom_port_;
  bool need_response_;
  T_ASC_Network* network;
};

//------------------------------------------------------------------------------

// At our end (C-STORE SCU) we add all supported
// contexts and let the SCP to select one.
OFCondition add_all_storage_presentation_contexts(T_ASC_Parameters *params) {
  // this would be the place to add support for compressed transfer syntaxes
  OFCondition cond = EC_Normal;

  int pid = 1;
  for (int i = 0; i < numberOfDcmLongSCUStorageSOPClassUIDs && cond.good(); i++) {
    if ((cond = ASC_addPresentationContext(params,
                                           pid,
                                           dcmLongSCUStorageSOPClassUIDs[i],
                                           _transfer_syntaxes,
                                           DIM_OF(_transfer_syntaxes))).bad()) {
      return cond;
    }

    pid += 2;  // Only odd presentation context id's
  }

  return cond;
}

//------------------------------------------------------------------------------

void request_association_to_storescp(const std::string& my_aet,
                                     const std::string& my_address,
                                     const std::string& remote_aet,
                                     const std::string& remote_address,
                                     T_ASC_Network* network,
                                     T_ASC_Association** assoc) {
  T_ASC_Parameters* params;
  CHK(ASC_createAssociationParameters(&params, ASC_DEFAULTMAXPDU));

  CHK(ASC_setPresentationAddresses(params, my_address.c_str(), remote_address.c_str()));
  CHK(ASC_setAPTitles(params, my_aet.c_str(), remote_aet.c_str(), NULL));

  CHK(add_all_storage_presentation_contexts(params));

  LOG(INFO) << "request_association_to_storescp";
  CHK(ASC_requestAssociation(network, params, assoc));
  std::cout << ">>> SUB ASSOC <<<" << std::endl;

  info_association_ac(*assoc);
}

//------------------------------------------------------------------------------

void storescu_store_single(T_ASC_Association* assoc,
                           const Image* image,
                           bool need_response,
                           const std::string& originator_aet,
                           DIC_US originator_id) try {
  if (VLOG_IS_ON(1)) TIMED_FUNC(timerObj);
  // DcmDataset* dataset = static_cast<DcmDataset*>(image->dataset_);

  // DIC_UI sopClass;
  // DIC_UI sopInstance;
  // CHK(DU_findSOPClassAndInstanceInDataSet(dataset, sopClass, sopInstance, false));
  LOG(INFO) << "sopClass: " << image->sop_class_uid_;
  LOG(INFO) << "sopInstance: " << image->sop_instance_uid_;

  // Which presentation context should be used.
  T_ASC_PresentationContextID presId = ASC_findAcceptedPresentationContextID(assoc, image->sop_class_uid_.c_str());

  LOG(INFO) << "Accepted presentation context id is:" << (unsigned int)presId;
  LOG(INFO) << "-----------------------------------------------";
  T_ASC_PresentationContext pc;
  ASC_findAcceptedPresentationContext(assoc->params, presId, &pc);
  ASC_dumpPresentationContext(&pc, std::cout);
  LOG(INFO) << "Accepted Transfer Syntax: " << pc.acceptedTransferSyntax;
  std::string xfer = pc.acceptedTransferSyntax;

  // Create subrequest
  T_DIMSE_C_StoreRQ request;
  request.MessageID = assoc->nextMsgID++;
  strncpy(request.AffectedSOPClassUID, image->sop_class_uid_.c_str(), DIC_UI_LEN);
  strncpy(request.AffectedSOPInstanceUID, image->sop_instance_uid_.c_str(), DIC_UI_LEN);
  request.DataSetType = DIMSE_DATASET_PRESENT;
  request.Priority = DIMSE_PRIORITY_HIGH;

  request.opts = (O_STORE_MOVEORIGINATORAETITLE | O_STORE_MOVEORIGINATORID);
  strncpy(request.MoveOriginatorApplicationEntityTitle,
          originator_aet.c_str(),
          DIC_AE_LEN);
  request.MoveOriginatorID = originator_id;

  /*
  T_DIMSE_C_StoreRSP rsp;
  DcmDataset* stDetail;

  CHK(DIMSE_storeUser(assoc,
                      presId,
                      &request,
                      0,  // We give no filename
                      dataset,
                      0,  // Not interested in moveSubOpProgressCallback,
                      0,  //     hence there's neither callback data given.
                      DIMSE_BLOCKING,
                      0,  // Not used in case of DIMSE_BLOCKING
                      &rsp,
                      &stDetail));
  */

  T_DIMSE_Message req;
  bzero(&req, sizeof(req));
  req.CommandField = DIMSE_C_STORE_RQ;
  request.DataSetType = DIMSE_DATASET_PRESENT;
  req.msg.CStoreRQ = request;

  VLOG(1) << "Sending image...";

  CHK(DIMSE_sendMessageUsingMemoryData(assoc,
                                       presId,
                                       &req,
                                       0,
                                       static_cast<DcmDataset*>(image->datasets_.at(pc.acceptedTransferSyntax)),
                                       0,
                                       0,
                                       0));
  
  // remember the ID of the presentation context in a local variable
  T_ASC_PresentationContextID thisPresId = presId;
  T_DIMSE_Message rsp;
  bzero(&rsp, sizeof(rsp));
  DcmDataset* stDetail;

  if (need_response) {
    CHK(DIMSE_receiveCommand(assoc,
                             DIMSE_BLOCKING,
                             0,
                             &thisPresId,
                             &rsp,
                             &stDetail,
                             0));
  VLOG(1) << "storescp response on image received.";
  }
} catch (const dicom_exception& ex) {
  // FIXME: Should set counting OK / NOK
  LOG(ERROR) << "Exception caught: " << ex.what() << "\n - " << ex.where();
}

//------------------------------------------------------------------------------

static void move_provider_callback(/* in */
                                   void* callbackData,
                                   OFBool cancelled,
                                   T_DIMSE_C_MoveRQ* request,
                                   DcmDataset* requestIdentifiers,
                                   int responseCount,
                                   /* out */
                                   T_DIMSE_C_MoveRSP* response,
                                   DcmDataset** stDetail,
                                   DcmDataset** responseIdentifiers) {
  if (cancelled) {
    response->DimseStatus = STATUS_MOVE_Cancel_SubOperationsTerminatedDueToCancelIndication;
    return;
  }

  CallbackData_SCP* callback_data = static_cast<CallbackData_SCP*>(callbackData);

  if (callback_data->store_assoc_ == 0) {
    // We have not established yet the sub-association to C-STORE SCP
    // so 1st we need to do that.
    // FIXME: This coould happen after we found what was requested and only offer that as presentation context
    request_association_to_storescp(callback_data->aet_,
                                    callback_data->host_,
                                    callback_data->raet_,
                                    callback_data->rhost_,
                                    callback_data->network_,
                                    &callback_data->store_assoc_);

    std::cout << "Request:" << std::endl;
    std::cout << "----------------------------------------------------------" << std::endl;
    requestIdentifiers->print(std::cout);
    std::cout << "----------------------------------------------------------" << std::endl;

    callback_data->response_ = new ImageStore::ImageMap();

    OFString qrlevel;
    requestIdentifiers->findAndGetOFString(DCM_QueryRetrieveLevel, qrlevel);

    if (qrlevel.compare("STUDY") == 0) {
      OFString study_uid;
      requestIdentifiers->findAndGetOFString(DCM_StudyInstanceUID, study_uid);

      auto imagemap_iterator = callback_data->imagestore_->study_map_.find(study_uid.c_str());
      callback_data->response_->insert(imagemap_iterator->second.cbegin(),
                                       imagemap_iterator->second.cend());
    } else if (qrlevel.compare("SERIES") == 0) {
      OFString series_uid;
      requestIdentifiers->findAndGetOFString(DCM_SeriesInstanceUID, series_uid);

      auto imagemap_iterator = callback_data->imagestore_->series_map_.find(series_uid.c_str());
      if (imagemap_iterator != callback_data->imagestore_->series_map_.end()) {
        callback_data->response_->insert(imagemap_iterator->second.cbegin(),
                                         imagemap_iterator->second.cend());
      }
    } else if (qrlevel.compare("IMAGE") == 0) {
      OFString instance_uid;
      requestIdentifiers->findAndGetOFString(DCM_SOPInstanceUID, instance_uid);

      auto image_iterator = callback_data->imagestore_->image_map_.find(instance_uid.c_str());
      if (image_iterator != callback_data->imagestore_->image_map_.end()) {
        callback_data->response_->insert(*image_iterator);
      }
    }

    callback_data->image_it_ = callback_data->response_->cbegin();
    callback_data->image_it_end_ = callback_data->response_->cend();

  } else {
    // Association to C-STORE SCP has been already established
    if (callback_data->multi_) {
      LOG(INFO) << "MULTI enabled";
      while (callback_data->image_it_ != callback_data->image_it_end_) {
        storescu_store_single(callback_data->store_assoc_,
                              callback_data->image_it_->second,
                              callback_data->need_response_,
                              callback_data->calling_aet_,
                              callback_data->originatorid_);
        ++callback_data->image_it_;
      }
    } else {
      storescu_store_single(callback_data->store_assoc_,
                            callback_data->image_it_->second,
                            callback_data->need_response_,
                            callback_data->calling_aet_,
                            callback_data->originatorid_);
      ++callback_data->image_it_;
    }
  }

  if (callback_data->image_it_ == callback_data->image_it_end_) {
    LOG(INFO) << "storescu done";
    response->DimseStatus = STATUS_Success;  // Success = End of store; Pending = will be more
    response->NumberOfRemainingSubOperations = 0;
    response->NumberOfCompletedSubOperations = 0;
    response->NumberOfFailedSubOperations = 0;
    response->NumberOfWarningSubOperations = 0;

    LOG(INFO) << "Releasing STORE association...";
    CHK(ASC_releaseAssociation(callback_data->store_assoc_));
    CHK(ASC_destroyAssociation(&callback_data->store_assoc_));
    LOG(INFO) << "Release STORE DONE.";
  } else {
    response->DimseStatus = STATUS_Pending;
  }
}

//------------------------------------------------------------------------------

void DicomDcmtk::process_request() {
  // T_ASC_Association* assoc;
  // CHK(ASC_receiveAssociation(network_, &assoc, ASC_DEFAULTMAXPDU));

  // When we open a socket connection to our SCP and close it without
  // sending any bytes DCMTK detects it as association. We need to check
  // if the assoc is actually complete.
  if (assoc_->params->DULparams.calledAPTitle[0] == 0) {
    LOG(ERROR) << "Called AET is empty, cleaning up and returning...";
    cleanup_f(assoc_);
    return;
  }

  if (negotiateAssociation(assoc_).bad()) {
    LOG(ERROR) << "Could not negotiate association, cleaning up and returning...";
    cleanup_f(assoc_);
    return;
  }

  CHK(ASC_acknowledgeAssociation(assoc_));
  info_association_ac(assoc_);

  while (1) {
    T_DIMSE_Message msg;
    T_ASC_PresentationContextID presID;
    OFCondition cond = DIMSE_receiveCommand(assoc_, DIMSE_BLOCKING, 0, &presID, &msg, NULL);

    if (cond.bad()) {
      if (cond == DUL_PEERREQUESTEDRELEASE) {
        LOG(INFO) << "Association Release";
        cond = ASC_acknowledgeRelease(assoc_);
      }
      else if (cond == DUL_PEERABORTEDASSOCIATION) {
        LOG(INFO) << "Association Aborted";
      }
      else  {
        OFString temp_str;
        LOG(ERROR) << "DIMSE failure (aborting association): " << DimseCondition::dump(temp_str, cond);
        /* some kind of error so abort the association */
        cond = ASC_abortAssociation(assoc_);
      }
      
      break;
    }

    switch (msg.CommandField) {
    case DIMSE_C_MOVE_RQ:
      LOG(INFO) << "C-MOVE request.";
      movescp_execute(msg, presID);
      break;
    case DIMSE_C_FIND_RQ:
      LOG(INFO) << "C-FIND request.";
      findscp_execute(msg, presID);
      break;
    default:
      LOG(ERROR) << "Cannot handle command: 0x"
                 << STD_NAMESPACE hex << (unsigned)msg.CommandField;
      break;
    }
  }
}

DcmElement* create_element(const std::string& str, const DcmTagKey& tag_key) {
  DcmElement* element = DcmItem::newDicomElement(tag_key);
  element->putString(str.c_str());
  return element;
}

void find_provider_callback(/* in */
                            void *callbackData,
                            OFBool cancelled,
                            T_DIMSE_C_FindRQ *request,
                            DcmDataset *requestIdentifiers,
                            int responseCount,
                            /* out */
                            T_DIMSE_C_FindRSP *response,
                            DcmDataset **responseIdentifiers,
                            DcmDataset **statusDetail) {

  VLOG(1) << "Find callback is called.";

  if (cancelled) {
    response->DimseStatus = STATUS_FIND_Cancel_MatchingTerminatedDueToCancelRequest;
    return;
  }
  
  CallbackData_SCP* callback_data = static_cast<CallbackData_SCP*>(callbackData);
    
  OFString qrlevel;
  requestIdentifiers->findAndGetOFString(DCM_QueryRetrieveLevel, qrlevel);

  if (responseCount == 1) {
    std::cout << "Request:" << std::endl;
    std::cout << "----------------------------------------------------------" << std::endl;
    requestIdentifiers->print(std::cout);
    std::cout << "----------------------------------------------------------" << std::endl;
    std::cout << "cancelled: " << cancelled << std::endl;
    std::cout << "----------------------------------------------------------" << std::endl;
    std::cout << "responseCount: " << responseCount << std::endl;
    std::cout << "----------------------------------------------------------" << std::endl;
  }

  if (qrlevel.compare("STUDY") == 0) {
    if (responseCount == 1) {
      LOG(INFO) << "QR level is STUDY";

      OFString study_uid;
      requestIdentifiers->findAndGetOFString(DCM_StudyInstanceUID, study_uid);
      
      callback_data->response_ = new ImageStore::ImageMap();

      ImageStore::HiStudyMap sm = callback_data->imagestore_->hi_study_map_;
      
      if (study_uid.empty() || study_uid.compare("*") == 0) {
        // Wildcard matching = we give back all series in study

        for (auto series_iterator : sm) {
          // FIXME check if image is there
          callback_data->response_->insert(*series_iterator.second.begin()->second.begin());
        }
        
        callback_data->image_it_ = callback_data->response_->cbegin();
        callback_data->image_it_end_ = callback_data->response_->cend();
      } else { // We only provide back one series that matches Series UID 

        auto series_iterator = sm.find(study_uid.c_str());
        if  (series_iterator == sm.end()) {
          response->DimseStatus = STATUS_Success;
          return;
        }

        // FIXME check if image is there
        callback_data->response_->insert(*series_iterator->second.begin()->second.begin());
        
        callback_data->image_it_ = callback_data->response_->cbegin();
        callback_data->image_it_end_ = callback_data->response_->cend();
      }
    }
 
    if (callback_data->image_it_ == callback_data->image_it_end_) {
      response->DimseStatus = STATUS_Success;
      return;
    }

    DcmDataset* dset = new DcmDataset;

    dset->insert(create_element(callback_data->image_it_->second->patient_name_, DCM_PatientName));
    dset->insert(create_element(callback_data->image_it_->second->patient_id_, DCM_PatientID));
    dset->insert(create_element(callback_data->image_it_->second->study_uid_.c_str(), DCM_StudyInstanceUID));
    // dset->insert(create_element(callback_data->image_it_->second->series_uid_.c_str(), DCM_SeriesInstanceUID));
    dset->insert(create_element(callback_data->image_it_->second->study_desc_, DCM_StudyDescription));
    // dset->insert(create_element(callback_data->image_it_->second->series_desc_, DCM_SeriesDescription));

    *responseIdentifiers = dset;
    response->DimseStatus = STATUS_Pending;
    ++callback_data->image_it_;
  } else if (qrlevel.compare("SERIES") == 0) {

    if (responseCount == 1) {
      LOG(INFO) << "QR level is SERIES";

      OFString study_uid;
      requestIdentifiers->findAndGetOFString(DCM_StudyInstanceUID, study_uid);
      OFString series_uid;
      requestIdentifiers->findAndGetOFString(DCM_SeriesInstanceUID, series_uid);
      
      callback_data->response_ = new ImageStore::ImageMap();

      ImageStore::HiStudyMap sm = callback_data->imagestore_->hi_study_map_;

      LOG(INFO) << "Series Instamnce UID: " << series_uid;
      if (series_uid.empty() || series_uid.compare("*") == 0) {
        // Wildcard matching = we give back all series in study

        auto study_iterator = sm.find(study_uid.c_str());
        if  (study_iterator == sm.end()) {
          response->DimseStatus = STATUS_Success;
          return;
        }

        LOG(INFO) << "SeriesMap size: " << study_iterator->second.size();
        for (auto image_iterator : study_iterator->second) {
          callback_data->response_->insert(*image_iterator.second.begin());
        }
        
        callback_data->image_it_ = callback_data->response_->cbegin();
        callback_data->image_it_end_ = callback_data->response_->cend();
      } else { // We only provide back one series that matches Series UID 

        auto study_iterator = sm.find(study_uid.c_str());
        if  (study_iterator == sm.end()) {
          response->DimseStatus = STATUS_Success;
          return;
        }

        auto series_iterator = study_iterator->second.find(series_uid.c_str());
        if  (series_iterator == study_iterator->second.end()) {
          response->DimseStatus = STATUS_Success;
          return;
        }
        
        auto image_iterator = series_iterator->second.begin();
        if  (image_iterator == series_iterator->second.end()) {
          response->DimseStatus = STATUS_Success;
          return;
        }

        callback_data->response_->insert(*image_iterator);
        
        callback_data->image_it_ = callback_data->response_->cbegin();
        callback_data->image_it_end_ = callback_data->response_->cend();
      }
    }
 
    if (callback_data->image_it_ == callback_data->image_it_end_) {
      response->DimseStatus = STATUS_Success;
      return;
    }

    DcmDataset* dset = new DcmDataset;

    dset->insert(create_element(callback_data->image_it_->second->patient_name_, DCM_PatientName));
    dset->insert(create_element(callback_data->image_it_->second->patient_id_, DCM_PatientID));
    dset->insert(create_element(callback_data->image_it_->second->study_uid_.c_str(), DCM_StudyInstanceUID));
    dset->insert(create_element(callback_data->image_it_->second->series_uid_.c_str(), DCM_SeriesInstanceUID));
    dset->insert(create_element(callback_data->image_it_->second->study_desc_, DCM_StudyDescription));
    dset->insert(create_element(callback_data->image_it_->second->series_desc_, DCM_SeriesDescription));

    *responseIdentifiers = dset;
    response->DimseStatus = STATUS_Pending;
    ++callback_data->image_it_;
  } else if (qrlevel.compare("IMAGE") == 0) {

    if (responseCount == 1) {
      LOG(INFO) << "QR level is IMAGE";

      OFString study_uid;
      requestIdentifiers->findAndGetOFString(DCM_StudyInstanceUID, study_uid);
      OFString series_uid;
      requestIdentifiers->findAndGetOFString(DCM_SeriesInstanceUID, series_uid);
      OFString instance_uid;
      requestIdentifiers->findAndGetOFString(DCM_SOPInstanceUID, instance_uid);

      callback_data->response_ = new ImageStore::ImageMap();

      if (instance_uid.empty() || instance_uid.compare("*") == 0) {
        // Wildcard matching = we give back all images in the series

        auto series_image_map_iterator =
          callback_data->imagestore_->series_map_.find(series_uid.c_str());
        if (series_image_map_iterator != callback_data->imagestore_->series_map_.end()) {
          callback_data->response_->insert(series_image_map_iterator->second.cbegin(),
                                           series_image_map_iterator->second.cend());
        }
        
        callback_data->image_it_ = callback_data->response_->cbegin();
        callback_data->image_it_end_ = callback_data->response_->cend();
      } else { // We only provide back one image that matches SOP Instance UID

        auto series_image_map_iterator =
          callback_data->imagestore_->series_map_.find(series_uid.c_str());
        if (series_image_map_iterator != callback_data->imagestore_->series_map_.end()) {
          auto image_iterator = series_image_map_iterator->second.find(instance_uid.c_str());

          if (image_iterator != series_image_map_iterator->second.end()) {
            callback_data->response_->insert(*image_iterator);
          }

          callback_data->image_it_ = callback_data->response_->cbegin();
          callback_data->image_it_end_ = callback_data->response_->cend();
        }
      }
    }
 
    if (callback_data->image_it_ == callback_data->image_it_end_) {
      response->DimseStatus = STATUS_Success;
      return;
    }

    DcmDataset* dset = new DcmDataset;

    dset->insert(create_element(callback_data->image_it_->first, DCM_SOPInstanceUID));
    dset->insert(create_element(callback_data->image_it_->second->sop_class_uid_, DCM_SOPClassUID));

    *responseIdentifiers = dset;
    ++callback_data->image_it_;
  } else {
    LOG(ERROR) << "Not supported QR level.";
  }

  response->DimseStatus = STATUS_Pending;
}

void DicomDcmtk::findscp_execute(T_DIMSE_Message& msg, T_ASC_PresentationContextID& presID) {
  LOG(INFO) << "C-FIND request indeed...";

  CallbackData_SCP callback_data;
  callback_data.imagestore_ = imagestore_;

  OFCondition cond = DIMSE_findProvider(assoc_,
                                        presID,
                                        &msg.msg.CFindRQ,
                                        find_provider_callback,
                                        &callback_data,
                                        DIMSE_BLOCKING,
                                        timeout_);
}

void DicomDcmtk::movescp_execute(T_DIMSE_Message& msg, T_ASC_PresentationContextID& presID) {
  T_DIMSE_C_MoveRQ* request = &msg.msg.CMoveRQ;
  std::string rhost;
  try {
    LOG(INFO) << "C-MOVE destination: " << request->MoveDestination;
    for (auto& r : *rhosts_) {
      LOG(INFO) << "C-MOVE rhosts: " << r.first << " - " << r.second;
    }
    rhost = rhosts_->at(request->MoveDestination);
  } catch (const std::out_of_range& ex) {
    LOG(WARNING) << "C-MOVE destinantion ("
                 << request->MoveDestination << ") is unknown. It shall be given in --rhosts";

    T_ASC_RejectParameters rej;
    rej.result = ASC_RESULT_REJECTEDPERMANENT;
    rej.source = ASC_SOURCE_SERVICEUSER;
    rej.reason = ASC_REASON_SU_CALLINGAETITLENOTRECOGNIZED;
    ASC_rejectAssociation(assoc_, &rej);
    cleanup_f(assoc_);
    return;
  }

  CallbackData_SCP callback_data;
  callback_data.network_ = network_;
  callback_data.move_assoc_ = assoc_;
  callback_data.store_assoc_ = 0;
  callback_data.imagestore_ = imagestore_;
  callback_data.aet_ = aet_;
  callback_data.host_ = dicom_host_;
  callback_data.calling_aet_ = assoc_->params->DULparams.callingAPTitle;
  callback_data.calling_host_ = assoc_->params->DULparams.callingPresentationAddress;
  callback_data.raet_ = request->MoveDestination;
  callback_data.rhost_ = rhost;
  callback_data.multi_ = multi_;
  callback_data.need_response_ = need_response_;
  callback_data.originatorid_ = request->MessageID;

  LOG(INFO) << "before moveProvider";
  CHK(DIMSE_moveProvider(assoc_,
                         presID,
                         &msg.msg.CMoveRQ,
                         move_provider_callback,
                         &callback_data,
                         DIMSE_BLOCKING,
                         timeout_));

  // cleanup_f(assoc_);
}


//------------------------------------------------------------------------------

void storescp_accept_association(T_ASC_Network* network,
                                 T_ASC_Association** assoc,
                                 const std::string& xfer) {
  LOG(INFO) << "Waiting for association...";
  int timeout = 1000;
  CHK(ASC_associationWaiting(network, timeout));

  LOG(INFO) << "accept store association started";
  CHK(ASC_receiveAssociation(network, assoc, ASC_DEFAULTMAXPDU));

  /*
  const char* ts[] = {UID_JPEGLSLosslessTransferSyntax,
                      UID_LittleEndianExplicitTransferSyntax,
                      UID_LittleEndianImplicitTransferSyntax,
                      UID_BigEndianExplicitTransferSyntax,
                      UID_JPEGProcess14SV1TransferSyntax};
  */

  const char* ts[] = {xfer.c_str()};
  int ts_size = DIM_OF(ts);
  CHK(ASC_acceptContextsWithPreferredTransferSyntaxes((*assoc)->params,
                                                      dcmAllStorageSOPClassUIDs,
                                                   // numberOfAllDcmStorageSOPClassUIDs
                                                      numberOfDcmAllStorageSOPClassUIDs,
                                                      ts,
                                                      ts_size));

  OFCondition cond = ASC_acknowledgeAssociation(*assoc);

  if (cond.bad()) {
    ASC_dropAssociation(*assoc);
    ASC_destroyAssociation(assoc);
  }
}

//------------------------------------------------------------------------------

bool storescp_exec_single(T_ASC_Network*, T_ASC_Association** assoc, bool need_response) {
  if (VLOG_IS_ON(1)) TIMED_FUNC(timerObj);
  T_DIMSE_Message msg;
  T_ASC_PresentationContextID presID;
  VLOG(1) << "before receiveCommand";
  OFCondition cond = DIMSE_receiveCommand(*assoc,
                                          DIMSE_BLOCKING,
                                          0,
                                          &presID,
                                          &msg,
                                          0);
  VLOG(1) << "after receiveCommand cond=" << cond.text();
  VLOG(1) << "MoveOriginatorApplicationEntityTitle: " << msg.msg.CStoreRQ.MoveOriginatorApplicationEntityTitle;
  VLOG(1) << "MoveOriginatorID: " << msg.msg.CStoreRQ.MoveOriginatorID;

  if (cond == EC_Normal && msg.CommandField == DIMSE_C_STORE_RQ) {
    T_DIMSE_C_StoreRQ *req;
    T_DIMSE_Message *msg1 = &msg;
    req = &msg1->msg.CStoreRQ;
    char imageFileName[PATH_MAX+1];
    snprintf(imageFileName,
             PATH_MAX,
             "%s.%s",
             dcmSOPClassUIDToModality(req->AffectedSOPClassUID),
             req->AffectedSOPInstanceUID);
    VLOG(1) << "received " << imageFileName;
    VLOG(1) << "before storeProvider " << imageFileName;

    DcmDataset* dset = 0;  // = dcmff.getDataset();
    /*
    cond = DIMSE_storeProvider(*assoc,
                               presID,
                               req,
                               0,
                               OFTrue,
                               &dset,
                               0,
                               0,
                               DIMSE_BLOCKING,
                               0);*/

    {
      if (VLOG_IS_ON(1)) TIMED_SCOPE(t2, "receiveDataSetInMemory");
      DIMSE_receiveDataSetInMemory(*assoc,
                                   DIMSE_BLOCKING,
                                   0,
                                   &presID,
                                   &dset,
                                   0,
                                   0);
    }

    T_DIMSE_C_StoreRSP response;
    bzero(&response, sizeof(response));
    response.DimseStatus = STATUS_Success;  /* assume */
    response.MessageIDBeingRespondedTo = req->MessageID;
    response.DataSetType = DIMSE_DATASET_NULL;  /* always for C-STORE-RSP */
    strncpy(response.AffectedSOPClassUID, req->AffectedSOPClassUID, DIC_UI_LEN);
    strncpy(response.AffectedSOPInstanceUID, req->AffectedSOPInstanceUID, DIC_UI_LEN);
    response.opts = (O_STORE_AFFECTEDSOPCLASSUID | O_STORE_AFFECTEDSOPINSTANCEUID);
    if (req->opts & O_STORE_RQ_BLANK_PADDING) response.opts |= O_STORE_RSP_BLANK_PADDING;
    if (dcmPeerRequiresExactUIDCopy.get()) response.opts |= O_STORE_PEER_REQUIRES_EXACT_UID_COPY;

    if (need_response) {
      DcmDataset* statusDetail = NULL;
      // if (VLOG_IS_ON(1)) PERFORMANCE_CHECKPOINT_WITH_ID(timerObj, "before sendStoreResponse");
      DIMSE_sendStoreResponse(*assoc,
                              presID,
                              req,
                              &response,
                              statusDetail);
      // if (VLOG_IS_ON(1)) PERFORMANCE_CHECKPOINT_WITH_ID(timerObj, "after sendStoreResponse");
    }

    DcmFileFormat file_format(dset);
    CHK(file_format.saveFile(imageFileName, dset->getCurrentXfer()));
    LOG(INFO) << "SAVED: " << imageFileName;

    VLOG(1) << "after storeProvider " << imageFileName;
    VLOG(1) << "cond= " << cond.text();

    E_TransferSyntax xfer = dset->getOriginalXfer();
    VLOG(1) << "Original TransferSyntax=" << xfer;
    OFString patientName;
    CHK(dset->findAndGetOFString(DCM_PatientName, patientName));
    VLOG(1) << "PN=" << patientName;
    OFString instance_uid;
    CHK(dset->findAndGetOFString(DCM_SOPInstanceUID, instance_uid));
    VLOG(1) << "SOPInstanceUID=" << instance_uid;
    OFString instance_number;
    CHK(dset->findAndGetOFString(DCM_InstanceNumber, instance_number));
    VLOG(1) << "InstanceNumber=" << instance_number;

    if (instance_number.compare("0") == 0) {
      // show_viewer(dset);
    }
  }

  /* clean up on association termination */
  if (cond == DUL_PEERREQUESTEDRELEASE) {
    cond = ASC_acknowledgeRelease(*assoc);
    ASC_dropSCPAssociation(*assoc);
    ASC_destroyAssociation(assoc);
    return false;
  } else {
    if (cond == DUL_PEERABORTEDASSOCIATION) {
    } else {
      if (cond != EC_Normal) {
        OFString temp_str;
        VLOG(1) << "DIMSE failure (aborting sub-association): "
                << DimseCondition::dump(temp_str, cond);
        /* some kind of error so abort the association */
        cond = ASC_abortAssociation(*assoc);
      }
    }
  }

  if (cond != EC_Normal) {
    // cleanup_f(*assoc);
  }

  return cond.good();
}

//------------------------------------------------------------------------------
/*
static void storescp_provider_callback(void* callback_data,
                                    T_ASC_Network* network,
                                    T_ASC_Association** subop_association) {
  VLOG(1) << "subop_provider_callback STARTED";

  LOG(INFO) << "Press - in PC1";
  // getchar();
  
  if (network == 0) {
    LOG(WARNING) << "aNet = 0, Can not work: returning from subop_provider_callback";
    return;
  }

  CallbackData_SCU* cbd = static_cast<CallbackData_SCU*>(callback_data);

  if (*subop_association == 0) {
    VLOG(1) << "we should acceptSubAssociation(aNet, subAssoc);";
    PERFORMANCE_CHECKPOINT((*cbd->timer_));
    std::string xfer = cbd->xfer_;
    storescp_accept_association(network, subop_association, xfer);
    PERFORMANCE_CHECKPOINT((*cbd->timer_));
  } else {
    // This will register in the store callbak
    VLOG(1) << "we should requestReceiverStoreSCP(subAssoc); " << *subop_association;
    storescp_exec_single(network, subop_association, cbd->need_response_);
  }

  VLOG(1) << "subop_provider_callback FINISHED";
}
*/

//------------------------------------------------------------------------------

T_ASC_Association* receive_association(CallbackData_SCU* cbd) {
  LOG(INFO) << "Waiting for association...";

  CHK(ASC_associationWaiting(cbd->network, INT_MAX));

  T_ASC_Association* assoc;
  LOG(INFO) << "accept store association started";
  CHK(ASC_receiveAssociation(cbd->network, &assoc, ASC_DEFAULTMAXPDU));
  OFString str;
  VLOG(1) << ASC_dumpParameters(str, assoc->params, ASC_ASSOC_RQ);

  const char* transfer_syntaxes[] = {cbd->xfer_.c_str()};
  LOG(INFO) << "XFER ====> " << cbd->xfer_.c_str();
  // const char* transfer_syntaxes[] = {UID_LittleEndianExplicitTransferSyntax,
  //                                    UID_BigEndianExplicitTransferSyntax,
  //                                    UID_LittleEndianImplicitTransferSyntax};

  int ts_size = DIM_OF(transfer_syntaxes);

  CHK(ASC_acceptContextsWithPreferredTransferSyntaxes(assoc->params,
                                                      dcmAllStorageSOPClassUIDs,
                                                      // numberOfAllDcmStorageSOPClassUIDs,
                                                      numberOfDcmAllStorageSOPClassUIDs,
                                                      transfer_syntaxes,
                                                      ts_size));

  VLOG(1) << ASC_dumpParameters(str, assoc->params, ASC_ASSOC_RQ);

  OFCondition cond_aa = ASC_acknowledgeAssociation(assoc);

  if (cond_aa.bad()) {
    LOG(ERROR) << "Accepting store association failed.";
    ASC_dropAssociation(assoc);
    ASC_destroyAssociation(&assoc);
  } else {
    LOG(INFO) << "store association accepted. Starting receiving images...";
  }

  return assoc;
}

void log_received_image(T_DIMSE_C_StoreRQ* req) {
     char imageFileName[PATH_MAX+1];
      snprintf(imageFileName,
               PATH_MAX,
               "%s.%s",
               dcmSOPClassUIDToModality(req->AffectedSOPClassUID),
               req->AffectedSOPInstanceUID);
      VLOG(1) << "received " << imageFileName;
      VLOG(1) << "before storeProvider " << imageFileName;
}

void send_response(T_ASC_Association* assoc, T_DIMSE_C_StoreRQ* req, T_ASC_PresentationContextID presID) {
  T_DIMSE_C_StoreRSP response;
  bzero(&response, sizeof(response));
  response.DimseStatus = STATUS_Success;  /* assume */
  response.MessageIDBeingRespondedTo = req->MessageID;
  response.DataSetType = DIMSE_DATASET_NULL;  /* always for C-STORE-RSP */
  strncpy(response.AffectedSOPClassUID, req->AffectedSOPClassUID, DIC_UI_LEN);
  strncpy(response.AffectedSOPInstanceUID, req->AffectedSOPInstanceUID, DIC_UI_LEN);
  response.opts = (O_STORE_AFFECTEDSOPCLASSUID | O_STORE_AFFECTEDSOPINSTANCEUID);
  if (req->opts & O_STORE_RQ_BLANK_PADDING) response.opts |= O_STORE_RSP_BLANK_PADDING;
  if (dcmPeerRequiresExactUIDCopy.get()) response.opts |= O_STORE_PEER_REQUIRES_EXACT_UID_COPY;

  DcmDataset* statusDetail = NULL;
  DIMSE_sendStoreResponse(assoc,
                          presID,
                          req,
                          &response,
                          statusDetail);
}

void log_received_image(DcmDataset* dset) {
  E_TransferSyntax xfer = dset->getOriginalXfer();
  VLOG(1) << "Original TransferSyntax=" << xfer;
  OFString patientName;
  CHK(dset->findAndGetOFString(DCM_PatientName, patientName));
  VLOG(1) << "PN=" << patientName;
  OFString instance_uid;
  CHK(dset->findAndGetOFString(DCM_SOPInstanceUID, instance_uid));
  VLOG(1) << "SOPInstanceUID=" << instance_uid;
  OFString instance_number;
  CHK(dset->findAndGetOFString(DCM_InstanceNumber, instance_number));
  VLOG(1) << "InstanceNumber=" << instance_number;
}

bool cleanup_assoc_needed(OFCondition cond_rc, T_ASC_Association* assoc) {
  /* clean up on association termination */

  OFCondition cond_release;
  if (cond_rc == DUL_PEERREQUESTEDRELEASE) {
    cond_release = ASC_acknowledgeRelease(assoc);
    LOG(INFO) << "DUL_PEERREQUESTEDRELEASE: "
              << "Releasing, dropping and destroying storescp association: "
              << cond_release.text();
    ASC_dropSCPAssociation(assoc);
    ASC_destroyAssociation(&assoc);
    return true;
  } else {
    if (cond_rc == DUL_PEERABORTEDASSOCIATION) {
      LOG(INFO) << "DUL_PEERABORTEDASSOCIATION: Nothing?.";
      // do nothing?
      return true;
    } else {
      if (cond_rc != EC_Normal) {
        OFString temp_str;
        VLOG(1) << "DIMSE failure (aborting sub-association): "
                << DimseCondition::dump(temp_str, cond_rc);
        /* some kind of error so abort the association */
        cond_release = ASC_abortAssociation(assoc);
        return true;
      }
    }
  }

  return false;
}

void serve_storescu(T_ASC_Association* assoc, CallbackData_SCU* cbd) {
  struct timespec tstart = {0, 0}, tend = {0, 0};
  clock_gettime(CLOCK_MONOTONIC, &tstart);

  size_t num_images_received = 0;
  size_t num_bytes_received = 0;
  while (true) {
    T_DIMSE_Message msg;
    T_ASC_PresentationContextID presID;
    VLOG(1) << "before receiveCommand";
    OFCondition cond_rc = DIMSE_receiveCommand(assoc,
                                               DIMSE_BLOCKING,
                                               0,
                                               &presID,
                                               &msg,
                                               0);
    VLOG(1) << "after receiveCommand cond=" << cond_rc.text();
    VLOG(1) << "MoveOriginatorApplicationEntityTitle: " << msg.msg.CStoreRQ.MoveOriginatorApplicationEntityTitle;
    VLOG(1) << "MoveOriginatorID: " << msg.msg.CStoreRQ.MoveOriginatorID;


    if (cond_rc == EC_Normal && msg.CommandField == DIMSE_C_STORE_RQ) {
      T_DIMSE_Message* msg1 = &msg;
      T_DIMSE_C_StoreRQ* req = &msg1->msg.CStoreRQ;
      log_received_image(req);

      DcmDataset* dset = 0;
      {
        if (VLOG_IS_ON(1)) TIMED_SCOPE(t2, "receiveDataSetInMemory");
        CHK(DIMSE_receiveDataSetInMemory(assoc,
                                         DIMSE_BLOCKING,
                                         0,
                                         &presID,
                                         &dset,
                                         0,
                                         0));
        num_images_received++;

        VLOG(1) << "Received dset length: " << dset->getLength(dset->getOriginalXfer());
        num_bytes_received += dset->getLength(dset->getOriginalXfer());
        log_received_image(req);

        char imageFileName[PATH_MAX+1];
        snprintf(imageFileName,
                 PATH_MAX,
                 "%s.%s",
                 dcmSOPClassUIDToModality(req->AffectedSOPClassUID),
                 req->AffectedSOPInstanceUID);

        DcmFileFormat file_format(dset);
        CHK(file_format.saveFile(imageFileName));
        LOG(INFO) << "SAVED: " << imageFileName;
      }

      if (cbd->need_response_)
        send_response(assoc, req, presID);
    }

    if (cleanup_assoc_needed(cond_rc, assoc)) break;
  }

  LOG(INFO) << "Number of images received: " << num_images_received;
  LOG(INFO) << "Number of bytes received: " << num_bytes_received;

  clock_gettime(CLOCK_MONOTONIC, &tend);
  double cstore_time =
    (static_cast<double>(tend.tv_sec) + 1.0e-9 * tend.tv_nsec) -
    (static_cast<double>(tstart.tv_sec) + 1.0e-9 * tstart.tv_nsec);

  LOG(INFO) << "store_scp DONE in " << cstore_time
            << " s; used bandwidth was " << num_bytes_received * 8 / cstore_time / 1.0e+6
            << " Mbit/s";
}

void* storescp_thread(void* pass) {
  CallbackData_SCU* cbd = reinterpret_cast<CallbackData_SCU*>(pass);

  while (true) {  // Infinite loop - when C-MOVE finishes it will exit program
    T_ASC_Association* assoc = receive_association(cbd);
    if (assoc)
      serve_storescu(assoc, cbd);
  }
}

void start_storescp(pthread_t* thread,
                    CallbackData_SCU* callback_data) {
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  T_ASC_Network* network = 0;
  CHK(ASC_initializeNetwork(NET_ACCEPTOR, callback_data->dicom_port_, INT_MAX, &network));
  callback_data->network = network;

  int rc = pthread_create(thread, &attr, storescp_thread, callback_data);
  if (rc != 0)
    LOG(ERROR) << "Could not create storescp thread.";

  pthread_attr_destroy(&attr);
}

//------------------------------------------------------------------------------

void DicomDcmtk::movescu_execute(const std::string& raet,
                                 const std::string& raddress,
                                 QueryLevel level,
                                 const std::string& study_uid,
                                 const std::string& series_uid,
                                 const std::string& instance_uid,
                                 const std::string& xfer,
                                 int message_id) {
  TIMED_FUNC(timerObj);

  struct timespec tstart = {0, 0}, tend = {0, 0};
  clock_gettime(CLOCK_MONOTONIC, &tstart);

  T_ASC_Parameters* params;
  CHK(ASC_createAssociationParameters(&params, ASC_DEFAULTMAXPDU));

  CHK(ASC_setAPTitles(params,
                      aet_.c_str(),  // calling
                      raet.c_str(),  // called
                      0));

  std::string ownaddress = dicom_host_;
  CHK(ASC_setPresentationAddresses(params,
                                   ownaddress.c_str(),  // Own (calling) ip:port
                                   raddress.c_str()));  // Remote (called) ip:port

  // This is for the C-MOVE SCU request, does not matter that much as the
  // C-STORE SCP accept part of the C-STORE negotiation.
  const char* ts[] = {UID_JPEGLSLosslessTransferSyntax,
                      UID_LittleEndianExplicitTransferSyntax,
                      UID_LittleEndianImplicitTransferSyntax,
                      UID_BigEndianExplicitTransferSyntax,
                      UID_JPEGProcess14SV1TransferSyntax};

  int ts_size = DIM_OF(ts);

  CHK(ASC_addPresentationContext(params,
                                 3,  // Why 3?
                                 UID_MOVEStudyRootQueryRetrieveInformationModel,
                                 ts,
                                 ts_size));

  T_ASC_Association* assoc;
  CHK(ASC_requestAssociation(network_, params, &assoc));
  PERFORMANCE_CHECKPOINT_WITH_ID(timerObj, "after ASC_requestAssociation");

  if (ASC_countAcceptedPresentationContexts(params) == 0) {
    THROW_DICOM_EXCEPTION("No presentation context was accepted.");
  }

  T_ASC_PresentationContextID presId =
    ASC_findAcceptedPresentationContextID(assoc,
                                          UID_MOVEStudyRootQueryRetrieveInformationModel);
  if ( presId == 0 ) {
    LOG(ERROR) << "No accepted presentation context.";
    // cleanup_f(assoc);
    return;
  }

  PERFORMANCE_CHECKPOINT_WITH_ID(timerObj, "after ASC_findAcceptedPresentationContextID");

  //-------------------------------------
  DcmElement* qrlevel_element = DcmItem::newDicomElement(DCM_QueryRetrieveLevel);
  switch (level) {
  case Dicom::SERIES:
    qrlevel_element->putString("SERIES"); break;
  case Dicom::STUDY:
    qrlevel_element->putString("STUDY"); break;
  case Dicom::IMAGE:
    qrlevel_element->putString("IMAGE"); break;
  default:
    LOG(ERROR) << "Not supported level: " << level;
    return;
  }

  DcmDataset dset;
  dset.insert(qrlevel_element, OFTrue);
  dset.insert(create_element(study_uid, DCM_StudyInstanceUID), OFTrue);
  dset.insert(create_element(series_uid, DCM_SeriesInstanceUID), OFTrue);
  dset.insert(create_element(instance_uid, DCM_SOPInstanceUID), OFTrue);

  //--------------------------------------

  // PERFORMANCE_CHECKPOINT_WITH_ID(timerObj, "after dset creation");

  T_DIMSE_C_MoveRQ req;
  if (message_id > 0) {
    req.MessageID = message_id;
  } else {
    req.MessageID = assoc->nextMsgID++;
  }
  req.Priority = DIMSE_PRIORITY_MEDIUM;
  req.DataSetType = DIMSE_DATASET_PRESENT;
  strncpy(req.AffectedSOPClassUID,
          UID_MOVEStudyRootQueryRetrieveInformationModel,
          DIC_UI_LEN);
  strncpy(req.MoveDestination, aet_.c_str(), DIC_AE_LEN);

  CallbackData_SCU callback_data;
  callback_data.timer_ = &timerObj;
  callback_data.xfer_ = xfer;
  callback_data.dicom_port_ = dicom_port_;
  callback_data.need_response_ = need_response_;

  DcmDataset*  rspIds = 0;
  DcmDataset*  statusDetail = 0;

  pthread_t thread;
  start_storescp(&thread, &callback_data);

  T_DIMSE_C_MoveRSP rsp;
  CHK(DIMSE_moveUser(  // in
                     assoc,
                     presId,
                     &req,
                     &dset,
                     0,  // moveuser_callback
                     0,
                     DIMSE_BLOCKING,
                     0,
                     network_,
                     0, // storescp_provider_callback,
                     0, // &callback_data,
                       // out
                     &rsp,
                     &statusDetail,
                     &rspIds,
                     OFTrue));
  // PERFORMANCE_CHECKPOINT_WITH_ID(timerObj, "after DIMSE_moveUser");
  LOG(INFO) << "after moveUser";

  CHK(ASC_releaseAssociation(assoc));

  clock_gettime(CLOCK_MONOTONIC, &tend);
  double cmove_time =
    ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - 
    ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec);

  LOG(INFO) << "movescu DONE in " << cmove_time << " s";

  // It's not the nicest way, but it does the job.
  // pthread_cancel(thread);
  // FIXME: Thread cancel created crash. We need to tell thread to stop.
  sleep(1);
  return;
}


//------------------------------------------------------------------------------
/*
#include <vtkSmartPointer.h>
#include <vtkImageImport.h>
#include <vtkBMPWriter.h>
#include <vtkImageViewer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkImageFlip.h>
#include <vtkImageResize.h>

void show_viewer(DcmDataset* dataset) {
  DicomDcmtk::init_codec();
  CHK(dataset->chooseRepresentation(EXS_LittleEndianExplicit, 0));

  LOG(INFO) << "Dataset=" << dataset;
  const Uint8* pixelData = 0;
  unsigned long num_elem = 0;
  CHK(dataset->findAndGetUint8Array(DCM_PixelData, pixelData, &num_elem));
  LOG(INFO) << "PixelData=" << pixelData;
  LOG(INFO) << "num_elem=" << num_elem;

  Uint16 rows;
  CHK(dataset->findAndGetUint16(DCM_Rows, rows));
  LOG(INFO) << "Rows=" << rows;

  Uint16 columns;
  CHK(dataset->findAndGetUint16(DCM_Columns, columns));
  LOG(INFO) << "Columns=" << columns;

  // Convert the c-style image to a vtkImageData
  vtkSmartPointer<vtkImageImport> imageImport = vtkSmartPointer<vtkImageImport>::New();
  imageImport->SetDataSpacing(1, 1, 1);
  imageImport->SetDataOrigin(0, 0, 0);
  imageImport->SetWholeExtent(0, columns-1, 0, rows-1, 0, 0);
  imageImport->SetDataExtentToWholeExtent();
  imageImport->SetDataScalarType(VTK_SHORT);
  imageImport->SetNumberOfScalarComponents(1);
  imageImport->SetImportVoidPointer((void*)pixelData);
  imageImport->Update();

  vtkSmartPointer<vtkImageFlip> flipYFilter = vtkSmartPointer<vtkImageFlip>::New();
  flipYFilter->SetFilteredAxis(1);  // flip y axis
  flipYFilter->SetInputConnection(imageImport->GetOutputPort());
  flipYFilter->Update();

  vtkSmartPointer<vtkImageResize> resize = vtkSmartPointer<vtkImageResize>::New();
  resize->SetInputConnection(imageImport->GetOutputPort());
  resize->SetOutputDimensions(512, (512.0 / columns) * rows, 1);
  resize->Update();
  vtkSmartPointer<vtkImageViewer> imageViewer = vtkSmartPointer<vtkImageViewer>::New();
  imageViewer->SetInputConnection(imageImport->GetOutputPort());

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();

  imageViewer->SetupInteractor(renderWindowInteractor);
  imageViewer->Render();
  imageViewer->GetRenderer()->ResetCamera();
  imageViewer->Render();

  renderWindowInteractor->Start();
}
*/
//------------------------------------------------------------------------------

// Only to study processing of compressed transfer syntaxes.
void processCompressed(DcmDataset* dset) {
  DcmElement* element = NULL;
  OFCondition result = dset->findAndGetElement(DCM_PixelData, element);
  if (result.bad()) {
    LOG(ERROR) << "Could not reach PixelData.";
  }

  LOG(INFO) << "Pixeldata is found.";

  //LOG(INFO) << element->getVRName();
        
  DcmPixelData *dpix = NULL;
  dpix = OFstatic_cast(DcmPixelData*, element);
  /* Since we have compressed data, we must utilize DcmPixelSequence
     in order to access it in raw format, e. g. for decompressing it
     with an external library.
  */
  E_TransferSyntax xferSyntax = EXS_Unknown;
  const DcmRepresentationParameter *rep = NULL;
  // Find the key that is needed to access the right representation of the data within DCMTK
  dpix->getOriginalRepresentationKey(xferSyntax, rep);

  LOG(INFO) << "Received dset length 2: " << dset->getLength(xferSyntax);
        
  // Access original data representation and get result within pixel sequence
  DcmPixelSequence *dseq = NULL;
  result = dpix->getEncapsulatedRepresentation(xferSyntax, rep, dseq);
  if ( result == EC_Normal ) {
    LOG(INFO) << "Pixel SEQ is found";
    DcmPixelItem* pixitem = NULL;
    // Access first frame (skipping offset table)
    dseq->getItem(pixitem, 1);
    if (pixitem == NULL) {
      LOG(ERROR) << "No frame.";
      return;
    }
    Uint8* pixData = NULL;
    // Get the length of this pixel item (i.e. fragment, i.e. most of the time, the lenght of the frame)
    Uint32 length = pixitem->getLength();
    if (length == 0) {
      LOG(ERROR) << "Pixitem length is 0.";
      return;
    }
    // Finally, get the compressed data for this pixel item
    result = pixitem->getUint8Array(pixData);
    // Do something useful with pixData...

    LOG(INFO) << "Pixdata size is: " << pixitem->getLength();
  }

  if (result.bad())
    LOG(ERROR) << "RETURN bad";
}
