#!/usr/bin/env python

import corr,time,struct,sys,logging,pylab,matplotlib
import mad_conf_parse
import os
import numpy as n

config_file='.'
fpga = []
roach='feng'
katcp_port=7147

def get_complex_fp(coeff, bitwidth, bp, signed=True):
    if signed:
        clipbits = bitwidth-1
    else:
        clipbits = bitwidth
    real = n.clip(n.round(n.real(coeff)*2**bp), -2**clipbits-1, 2**clipbits-1)
    imag = n.clip(n.round(n.imag(coeff)*2**bp), -2**clipbits-1, 2**clipbits-1)
    return n.array(real+1j*imag)

def get_real_fp(coeff, bitwidth, bp, signed=False):
    if signed:
        clipbits = bitwidth-1
    else:
        clipbits = bitwidth
    return n.clip(n.round(n.real(coeff)*2**bp), -2**clipbits-1, 2**clipbits-1)

def modify_ampcoeffs(ant, pol, cal_coeffs, closed_loop=True, verbose=False):
    """Multiply the current coefficients by a constant, or a vector, or replace them with a new calibration set"""
    nchans=1024
    nants=32
    npols=1
    decimation=2
    flip_spectrum=False
    ncoeffs = nchans/decimation
    coeff = n.ones((nants,npols,nchans/decimation),dtype=float)
    if n.size(cal_coeffs) == 1:
        #print 'OCCHIOOOO'
        cal_coeffs = n.array([cal_coeffs], dtype=float)
    if len(cal_coeffs) == len(coeff[ant,pol]):
        # Check the number of calibration coefficients is the same as the number of coefficients already associated with the manager instance
        dec_coeffs = n.array(cal_coeffs, dtype=float)
    elif len(cal_coeffs) == nchans:
        #if it isn't, but there are the same number of calibration coeffs as there are channels, then decimate and apply the calibration
        dec_coeffs = n.zeros_like(coeff[ant,pol])
        #print 'len(dec_coeffs)', len(dec_coeffs)
        for i in range(ncoeffs):
            dec_coeffs[i] = n.average(cal_coeffs[i*decimation:(i+1)*decimation])
    elif len(cal_coeffs) == 1:
        #if there's only one calibration coefficient, apply it to all the channels
        dec_coeffs = n.ones_like(coeff[ant,pol])*cal_coeffs[0]
    else:
        raise IndexError('''The number of calibration coefficients don\'t seem to be the number
                         or frequency channels, the number of decimated channels or a single value. I have
                         no idea what to do!''')
    if closed_loop is False:
        coeff[ant][pol] = dec_coeffs
        if verbose:
            print 'AMP EQ: ant %d, pol%d' %(ant,pol),
            print ' ABS',
            print n.abs(coeff[ant][pol][0]),
            print ' PHASE',
            print n.angle(coeff[ant][pol][0])
    else:
        if verbose:
            print 'old coeff:', coeff[ant,pol]
            print 'calibration:', dec_coeffs
        coeff[ant,pol] = coeff[ant,pol] * dec_coeffs
        if verbose:
            print 'newcoeff:', coeff[ant,pol]
    return coeff[ant]

def modify_phscoeffs(ant, pol, cal_coeffs, closed_loop=True, verbose=False):
    nchans=1024
    nants=32
    npols=1
    decimation=2
    flip_spectrum=False
    ncoeffs = nchans/decimation
    coeff = n.ones((nants,npols,nchans/decimation),dtype=complex)
    if n.size(cal_coeffs) == 1:
        #print 'OCCHIOOOO'
        cal_coeffs = n.array([cal_coeffs], dtype=complex)
    if len(cal_coeffs) == len(coeff[ant,pol]):
        # Check the number of calibration coefficients is the same as the number of coefficients already associated with the manager instance
        dec_coeffs = n.array(cal_coeffs, dtype=complex)
    elif len(cal_coeffs) == nchans:
        #if it isn't, but there are the same number of calibration coeffs as there are channels, then decimate and apply the calibration
        dec_coeffs = n.zeros_like(coeff[ant,pol])
        #print 'len(dec_coeffs)', len(dec_coeffs)
        for i in range(ncoeffs):
            dec_coeffs[i] = n.average(cal_coeffs[i*decimation:(i+1)*decimation])
    elif len(cal_coeffs) == 1:
        #if there's only one calibration coefficient, apply it to all the channels
        dec_coeffs = n.ones_like(coeff[ant,pol])*cal_coeffs[0]
    else:
        raise IndexError('''The number of calibration coefficients...''')        
    if closed_loop is False:
        coeff[ant][pol] = dec_coeffs
        if verbose:
            print 'PHS EQ: ant %d, pol%d' %(ant,pol),
            print ' ABS',
            print n.abs(coeff[ant][pol][0]),
            print ' PHASE',
            print n.angle(coeff[ant][pol][0])
    else:
        if verbose:
            print 'old coeff:', coeff[ant,pol]
            print 'calibration:', dec_coeffs
        coeff[ant,pol] = coeff[ant,pol] * dec_coeffs
        if verbose:
            print 'newcoeff:', coeff[ant,pol]
    return coeff[ant]

def eq_write_all_phs(fpga,new_phs_coeffs, verbose=False):
    """Write to Phase BRAM the equalization coefficents for a given antpol on the F Engine"""
    coeffs = get_complex_fp(new_phs_coeffs,16,15)
    if verbose:
        print 'Coefficient 256 for all pols'
        print coeffs[:,:,256]
        print 'ABS'
        print n.abs(coeffs[:,:,256])
        print 'PHASE (degrees)'
        print n.angle(coeffs[:,:,256])*180./n.pi
    uints = ((n.array(coeffs.real,dtype=int)&0xffff)<<16)+(n.array(coeffs.imag,dtype=int)&0xffff)
    MAP = [0,4,1,5,2,6,3,7]
    #TODO: we use n_ants_sp here as the number of ants per fpga, and the TOTAL number of ants.
    #In general these are not the same. Fix.
    for pn,pol in enumerate(['x','y'][0:1]):
        for eq_subsys in range(32//8):
            bin_str = ''
            for ant_mux_index in range(8):
                offset = 1024/2*MAP*4 #Offset in bytes, not words
                if verbose:
                    print '(ant %d%s): Packing %d coefficients to be written to ram %d' %(MAP[ant_mux_index]+8*eq_subsys,pol,len(uints[MAP[ant_mux_index],pn]),eq_subsys)
                bin_str = bin_str + n.array(uints[MAP[ant_mux_index]+8*eq_subsys,pn],dtype='>u4').tostring()
                if verbose:
                    print 'Coefficients as packed:'
                    print uints[MAP[ant_mux_index]+8*eq_subsys,pn][0]
                #for uint in uints[ant,pn]:
                #    bin_str = bin_str + struct.pack('>L', uint)
            if verbose:
                print 'Writing %d coeffs to phase_EQ%d_coeff_bram' %(len(bin_str)/4,eq_subsys)
            fpga.write('phase_EQ%d_coeff_bram' %eq_subsys, bin_str)
            time.sleep(0.2)

def eq_write_all_amp(fpga,new_amp_coeffs,verbose=False):
    """Write to Amplitude BRAM the equalization coefficents for a given antpol on the F Engine"""
    coeffs = get_real_fp(new_amp_coeffs, 32,16,signed=False)
    if verbose:
        print 'Coefficient 256 for all pols'
        print coeffs[:,:,256]
        print 'ABS'
        print n.abs(coeffs[:,:,256])
    uints = n.array(coeffs,dtype=n.uint32)
    MAP = [0,4,1,5,2,6,3,7]
    #TODO: we use n_ants_sp here as the number of ants per fpga, and the TOTAL number of ants.
    #In general these are not the same. Fix.
    for pn,pol in enumerate(['x','y'][0:1]):
        for eq_subsys in range(32//8):
            bin_str = ''
            for ant_mux_index in range(8):
                offset = 1024/2*MAP*4 #Offset in bytes, not words
                if verbose:
                    print '(ant %d%s): Packing %d coefficients to be written to ram %d' %(MAP[ant_mux_index]+8*eq_subsys,pol,len(uints[MAP[ant_mux_index],pn]),eq_subsys)
                bin_str = bin_str + n.array(uints[MAP[ant_mux_index]+8*eq_subsys,pn],dtype='>u4').tostring()
                if verbose:
                    print 'Coefficients as packed:'
                    print uints[MAP[ant_mux_index]+8*eq_subsys,pn][0]
                #for uint in uints[ant,pn]:
                #    bin_str = bin_str + struct.pack('>L', uint)
            if verbose:
                print 'Writing %d coeffs to amp_EQ%d_coeff_bram' %(len(bin_str)/4,eq_subsys)
            fpga.write('amp_EQ%d_coeff_bram' %eq_subsys, bin_str)
            time.sleep(0.2)

if __name__ == '__main__':
    from optparse import OptionParser

    p = OptionParser()
    p.set_usage('gaincal_load.py [options] INST_CONFIG_FILE')
    p.set_description(__doc__)
    p.add_option('-v', '--verbose', dest='verbose', action='store_true', default=False,
            help='Print lots of lovely debug information')
    p.add_option('-c', '--closed_loop', dest='closed_loop',action='store_true', default=False,
            help='Use this flag to modify (by multiplication), rather than replace, the current calibration coefficient set')
    p.add_option('-A', '--amp_cal', dest='amp_cal', default=None,
            help='Use this flag to specify the amplitude calibratiion .txt file')
    p.add_option('-P', '--phs_cal', dest='phs_cal', default=None,
            help='Use this flag to specify the phase calibratiion .txt file')

    opts, args = p.parse_args(sys.argv[1:])

    #if len(args)<1:
    #    print 'Please specify an instrument configuration file! \nExiting.'
    #    exit()

    if opts.amp_cal is None:
        print 'No amplitude calibration file specified. Specify one with the -A flag!'
        exit()

    if opts.phs_cal is None:
        print 'No phase calibration file specified. Specify one with the -P flag!'
        exit()

    #print 'Loading Instrument config file (%s) and connecting...' %(args[0])
    #feng=poxy.medInstrument.fEngine(args[0],program=False)
    #feng.eq_init_phs(load_pickle=True)
    #feng.eq_init_amp(load_pickle=True)
    #fConf = feng.fConf
    n_ants = 32#fConf.n_ants

    print '\nLoading Amp calibration file: %s' %opts.amp_cal
    ampfile = open(opts.amp_cal,'r')
    coeffs = ampfile.readlines()
    amp_coeffs = n.zeros([32,1024],dtype=float) #32 antennas, 1024 chans
    valori = []
    indice = []
    for coeff in coeffs:
        c = coeff[:-1].split()
        #print c
        #c is ['aXX','1.234'] pairs
        index = int(c[0].lstrip('a'))
        val = float(c[1])
        indice += [index]
        valori += [val]
        amp_coeffs[index,:] = val #give all channels the same coeff
    amp_header = n.zeros([32],dtype=float)
    for a in range(len(indice)):
        amp_header[indice[a]] = valori[a]
    print 'done'

    print '\nLoading phase calibration file: %s' %opts.phs_cal
    phsfile = open(opts.phs_cal,'r')
    coeffs = phsfile.readlines()
    phs_coeffs = n.zeros([32,1024],dtype=complex) #32 ants, 1024 chans
    valori = []
    indice = []
    for coeff in coeffs:
        c = coeff[:-1].split()
        #c is ['aXX','1.234'] pairs
        index = int(c[0].lstrip('a'))
        val = n.exp(1j*float(c[1])*n.pi/180.)
        phs_coeffs[index,:] = val #give all channels the same coeff
        indice += [index]
        valori += [float(c[1])]
    phs_header = n.zeros([32],dtype=float) #32 ants
    for a in range(len(indice)):
        phs_header[indice[a]] = valori[a]
    print 'done'

    for ant in range(32):
        print "Antenna %2d, (channel 0): AMPLITUDE: %3.2f\tPHASE (degs): %3.1f"%(ant, amp_coeffs[ant,0], 180./n.pi*n.angle(phs_coeffs[ant,0]))
    
    # Map the antenna numberings used for the coefficients to the numberings used for the f-engine
    # here we assume they are the same
    ant_remap = n.arange(32)

#    for ant in range(fConf.n_ants_sp):
    new_phs_coeffs = n.ones((32,1,512),dtype=complex)
    new_amp_coeffs = n.ones((32,1,512),dtype=float)
    for ant in range(32):
        print 'Modifying calibration coefficients for antenna %d' %ant
        #reverse coefficient channels because medicina spectrum is inverted
        #print len(phs_coeffs),len(phs_coeffs[ant_remap[ant],::-1])
        new_phs_coeffs[ant]=modify_phscoeffs(ant,0,phs_coeffs[ant_remap[ant],::-1],closed_loop=opts.closed_loop, verbose=opts.verbose)
        new_amp_coeffs[ant]=modify_ampcoeffs(ant,0,amp_coeffs[ant_remap[ant],::-1],closed_loop=opts.closed_loop, verbose=opts.verbose)
        #print 'Nuovo Amp coeff ',new_amp_coeffs[ant][0][0]

    print('\nConnecting to ROACH board named "%s"... '%roach),
    fpga = corr.katcp_wrapper.FpgaClient(roach)
    time.sleep(1)
    if fpga.is_connected():
        print 'ok'
    else:
        print 'ERROR connecting to server %s on port %i.\n'%(roach,katcp_port)
        exit()

    print '\n Writing phase coefficients...',
    eq_write_all_phs(fpga,new_phs_coeffs, verbose=opts.verbose)
    print 'done'
    print ' Updating phase coefficients on header bram...',
    valore=phs_header.tolist()
    #print valore
    val = ''
    for i in range(len(valore)):
        val+=struct.pack('>f',valore[i])
    fpga.write('header',val,408)
    print 'done'

    print '\n Writing amp coefficients...',
    eq_write_all_amp(fpga,new_amp_coeffs, verbose=opts.verbose)
    print 'done'

    print ' Updating amp coefficients on header bram...',
    #valore=struct.pack('>32L',amp_header.tolist())
    valore=amp_header.tolist()
    val = ''
    for i in range(len(valore)):
        val+=struct.pack('>f',valore[i])
        
    #print valore
    #print amp_header
    fpga.write('header',val,280)
    print 'done'
    #print phs_header
