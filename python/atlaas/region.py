import gdal
import numpy
import Image # aka PIL, because gdal PNG driver does not support WriteBlock

def merge(filetmp, filedst):
    gtmp = gdal.Open(filetmp)
    gdst = gdal.Open(filedst)
    r1 = gtmp.ReadAsArray()
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
    logger.debug("merge coverage gain: %.3f" %
        (coverage - float(gdst.GetMetadata().get('COVERAGE', '0'))))
    gdst.SetMetadataItem('COVERAGE', str(coverage))
    gdst.SetMetadataItem('AVGALPHA', str(avgalpha))
