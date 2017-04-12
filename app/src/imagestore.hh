// Copyright 2015 Tamas Palagyi
#ifndef APP_SRC_IMAGESTORE_H_
#define APP_SRC_IMAGESTORE_H_

#include <map>
#include <string>

#include "dicom.hh"

class ImageStore {
 
  std::string rootdir_;

  void fill_study_map(const std::string& path, const Xfers& xfers);
  
 public:

  // All images map
  typedef std::map<SOPInstanceUID, Image*> ImageMap;
  ImageMap image_map_;

  // All series map
  typedef std::map<SeriesUID, ImageMap> SeriesMap;
  SeriesMap series_map_;

  // All studies map
  typedef std::map<StudyUID, ImageMap> StudyMap;
  StudyMap study_map_;
  
  // Hierarchical studies map
  //typedef std::map<StudyUID, SeriesMap> HiStudyMap;
  typedef std::map<StudyUID, SeriesMap> HiStudyMap;
  HiStudyMap hi_study_map_;
  
  explicit ImageStore(const std::string& path, const Xfers& xfers);
  void print() const;
};

#endif  // APP_SRC_IMAGESTORE_H_
