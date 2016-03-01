import os
import shutil
import gdal
import numpy
import Image # aka PIL, because gdal PNG driver does not support WriteBlock

def merge(filesrc, filedst):
    gsrc = gdal.Open(filesrc)
    gdst = gdal.Open(filedst)
    r1 = gsrc.ReadAsArray()
    r2 = gdst.ReadAsArray()
    # compute cost from r1 and r2 ([0]: cost, [1]: precision)
    # cost is cost_r1 where precision_r1 > precision_r2 else cost_r2
    cost = numpy.where(r1[1] > r2[1], r1[0], r2[0])
    alph = numpy.where(r1[1] > r2[1], r1[1], r2[1])
    image = Image.fromarray(cost)
    ialph = Image.fromarray(alph)
    image.putalpha(ialph)
    image.save(filedst)
    # update COVERAGE metadata
    coverage = alph[alph > 0].size / float(alph.size)
    avgalpha = numpy.average(alph)
    gdst.SetMetadataItem('COVERAGE', str(coverage))
    gdst.SetMetadataItem('AVGALPHA', str(avgalpha))
    return coverage, avgalpha

def merge_or_copy(filesrc, filedst):
    if os.path.isfile(filedst):
        merge(filesrc, filedst)
    else: # copy temp to dest
        # os.rename dont work across filesystem
        #   [Errno 18] Invalid cross-device link
        shutil.copy(filesrc, filedst)
        shutil.copy(filesrc+".aux.xml", filedst+".aux.xml")
