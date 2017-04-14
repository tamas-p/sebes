// Copyright 2015 Tamas Palagyi
//------------------------------------------------------------------------------

#include "imagestore.hh"

// Standard
#include <stdio.h>
#include <dirent.h>
#include <openssl/md5.h>

// Logging
#include <easylogging++.h>

#include "dicom.hh"
#include "dcmtk.hh"

//------------------------------------------------------------------------------

void list_dir(const char* path, std::vector<std::string>* found) {
  struct dirent* entry;
  DIR* dp;

  dp = opendir(path);
  if (dp == 0) {
    // PLOG(ERROR) << path;
    return;  // Exception
  }

  while ((entry = readdir(dp))) {
    char newpath[PATH_MAX];
    int len = snprintf(newpath, sizeof(newpath)-1, "%s/%s", path, entry->d_name);
    newpath[len] = 0;

    if (entry->d_type == DT_DIR) {
      if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;
      list_dir(newpath, found);
    }

    // LOG(INFO) << "Found " << newpath;
    found->push_back(newpath);
  }

  closedir(dp);
}

//------------------------------------------------------------------------------

ImageStore::ImageStore(const std::string& path, const Xfers& xfers) : rootdir_(path) {
  fill_study_map(path, xfers);
}

//------------------------------------------------------------------------------

void calculate_checksum(void* buffer, size_t length) {
  MD5_CTX mdContext;
  MD5_Init(&mdContext);
}

//------------------------------------------------------------------------------

void ImageStore::fill_study_map(const std::string& path, const Xfers& xfers) {
  std::vector<std::string>* found = new std::vector<std::string>();
  list_dir(path.c_str(), found);

  DicomDcmtk::init_codec();

  for (auto& it : *found) {
    Image* image = new Image();

    try {
      DicomDcmtk::load_image_file(it, image, xfers);  // FIXME, no dcmtk here
    } catch (const dicom_exception& ex) {
      continue;
    }

    // register image into image_map_
    image_map_[image->sop_instance_uid_] = image;

    // register image into series_map_
    series_map_[image->series_uid_][image->sop_instance_uid_] = image;

    // register image into study_map_
    study_map_[image->study_uid_][image->sop_instance_uid_] = image;

    // register image into hi_study_map_
    hi_study_map_[image->study_uid_][image->series_uid_][image->sop_instance_uid_] = image;

    LOG(INFO) << "Added " << image->source_
              << " - Xfer: " << image->transfer_syntax_uid_
              << " - Dset: " << image->dataset_
              << " - [" << image->study_uid_
              << "][" << image->series_uid_
              << "][" << image->sop_instance_uid_ << "]";
  }
}

//------------------------------------------------------------------------------

void ImageStore::print() const {
  std::cout << "study_map_ size=" << study_map_.size() << std::endl;;
  for (auto& it_st : study_map_) {
    std::cout << "  study_uid=" << it_st.first;
    std::cout << " number of images in this study = " << it_st.second.size() << std::endl;
    for (auto& it_im : it_st.second) {
      Image* image = it_im.second;
      std::cout  << ">Image " << image->source_
                 << " - Xfer: " << image->transfer_syntax_uid_
                 << " - Dset: " << image->dataset_
                 << " - [" << image->study_uid_
                 << "][" << image->series_uid_
                 << "][" << image->sop_instance_uid_ << "]" << std::endl;
    }
  }

  std::cout << "series_map_ size=" << series_map_.size() << std::endl;;
  for (auto& it_se : series_map_) {
    std::cout << "  series_uid=" << it_se.first;
    std::cout << " number of images in this series = " << it_se.second.size() << std::endl;
    for (auto& it_im : it_se.second) {
      Image* image = it_im.second;
      std::cout  << ">Image " << image->source_
              << " - Xfer: " << image->transfer_syntax_uid_
              << " - Dset: " << image->dataset_
              << " - [" << image->study_uid_
              << "][" << image->series_uid_
                 << "][" << image->sop_instance_uid_ << "]" << std::endl;
    }
  }

  std::cout << "image_map_ size=" << image_map_.size() << std::endl;;

  std::cout << "hi_study_map_ size=" << hi_study_map_.size() << std::endl;;
  for (auto& it_sm : hi_study_map_) {
    std::cout << "  study_uid=" << it_sm.first << std::endl;
    std::cout << "  number of series in this study = " << it_sm.second.size() << std::endl;
    for (auto& it_serm : it_sm.second) {
      std::cout << "    series_uid = " << it_serm.first << std::endl;
      std::cout << "    number of images in this series = " << it_serm.second.size() << std::endl;
      for (auto& it_im : it_serm.second) {
      Image* image = it_im.second;
      std::cout  << ">Image " << image->source_
              << " - Xfer: " << image->transfer_syntax_uid_
              << " - Dset: " << image->dataset_
              << " - [" << image->study_uid_
              << "][" << image->series_uid_
                 << "][" << image->sop_instance_uid_ << "]" << std::endl;
      }
    }
  }
}

//------------------------------------------------------------------------------
