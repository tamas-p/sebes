# sebes

./sebes --host=sebes@127.0.0.1:1111 --rhosts=huz@127.0.0.1:2222 --imagedir=/store/dicom/db/COMMON/PERF1000/

./huz --v=0 --host=huz@127.0.0.1:2222 --rhost=sebes@127.0.0.1:1111 --xfer=1.2.840.10008.1.2.4.70 --study=1.2.3.4.5