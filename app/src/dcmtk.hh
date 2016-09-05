// Copyright 2016 Tamas Palagyi
//------------------------------------------------------------------------------
#ifndef APP_SRC_DCMTK_H_
#define APP_SRC_DCMTK_H_
//------------------------------------------------------------------------------

#include "dicom.hh"

// DCMTK
#include <dcmtk/dcmnet/assoc.h>

//------------------------------------------------------------------------------

#define CHK(a) chk_cond(__FILE__, __LINE__, a)
void chk_cond(const char* file, int line, OFCondition cond);
void chk_cond(const char* file, int line, OFBool res);

//------------------------------------------------------------------------------

void show_viewer(DcmDataset* dataset);

class DicomDcmtk : public Dicom {
  T_ASC_Network* network_;
  T_ASC_Association* assoc_;
  
public:

  explicit DicomDcmtk(int port,
                      const std::string& aet,
                      const ImageStore* imagestore,
                      const AetMap* rhosts,
                      bool multi,
                      bool need_response);
  explicit DicomDcmtk(const std::string& aet,
                      const std::string& host,
                      int port,
                      bool need_response);
 
  static void load_image_file(const std::string& path, Image* image);
  virtual void init();
  static void init_codec();
  virtual void waitfor_association();
  virtual void movescp_execute();

  virtual void movescu_execute(const std::string& raet,
                               const std::string& raddress,
                               Dicom::QueryLevel level,
                               const std::string& dicom_uid,
                               const std::string& xfer);

  virtual void cleanup();
};

//------------------------------------------------------------------------------
#endif  // APP_SRC_DCMTK_H_
