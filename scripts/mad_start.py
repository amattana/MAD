#!/usr/bin/env python

import corr,time,numpy,struct,sys,logging,pylab,matplotlib
import mad_conf_parse
import os

config_file='.'
fpga = []

def exit_fail():
    print 'FAILURE DETECTED. Log entries:\n',lh.printMessages()
    try:
        fpga.stop()
    except: pass
    raise
    exit()

def exit_clean():
    try:
        fpga.stop()
    except: pass
    exit()

def check_adc_sync():
    rv = fpga.read_uint('adc_sync_test')
    while (rv&0b111) != 1:
        fpga.write_int('adc_spi_ctrl', 1)
        time.sleep(.05)
        fpga.write_int('adc_spi_ctrl', 0)
        time.sleep(.05)
        print '    ERROR: adc sync test returns %i'%rv
        rv = fpga.read_uint('adc_sync_test')
    print '    SUCCESS: adc sync test returns %i (1 = ADC syncs present & aligned)' %rv

def bit_string(val, width):
    bitstring = ''
    for i in range(width):
        bitstring += str((val & (1<<i))>>i)
    return bitstring

def adc_cal(calreg='x64_adc_ctrl', debug=False):
    # Some Addresses...
    CTRL       = 0
    DELAY_CTRL = 0x4
    DATASEL    = 0x8
    DATAVAL    = 0xc

    for j in range(0,8):
        if debug: print '%d: '%(j)
        #select bit
        fpga.blindwrite(calreg, '%c%c%c%c'%(0x0,0x0,0x0,j//2), DATASEL)
        #reset dll
        fpga.blindwrite(calreg, '%c%c%c%c'%(0x0,0x0,0x0,(1<<j)), DELAY_CTRL)
        if debug: print "ready\tstable\tval0"
        stable = 1
        prev_val = 0
        while(stable==1):
            fpga.blindwrite(calreg, '%c%c%c%c'%(0x0,0xff,(1<<j),0x0), DELAY_CTRL)
            #val = numpy.fromstring(fpga.read(calreg,4,DATAVAL), count=4, dtype='uint8')
            val    = struct.unpack('>L', (fpga.read(calreg,4,DATAVAL)))[0]
            val0   = (val & ((0xffff)<<(16*(j%2))))>>(16*(j%2))
            stable = (val0&0x1000)>>12
            ready  = (val0&0x2000)>>13
            fclk_sampled = bit_string((val0&0x0fff),12)
            if val0 != prev_val and prev_val != 0:
                break
            prev_val = val0
            if debug: print '%d\t%d\t%s' %(ready, stable, fclk_sampled)
        if debug: print ''
        for i in range(10):
            fpga.blindwrite(calreg, '%c%c%c%c'%(0x0,0xff,(1<<j),0x0), DELAY_CTRL)
            #val = numpy.fromstring(fpga.read(calreg,4,DATAVAL), count=4, dtype='uint8')
            val    = struct.unpack('>L', (fpga.read(calreg,4,DATAVAL)))[0]
            val0   = (val & ((0xffff)<<(16*(j%2))))>>(16*(j%2))
            stable = (val0&0x1000)>>12
            ready  = (val0&0x2000)>>13
            fclk_sampled = bit_string((val0&0x0fff),12)
            if debug: print '%d\t%d\t%s' %(ready, stable, fclk_sampled)
        if debug:print ''

def prog_feng(roach,bitstream):
    print '   Deprogramming FPGAs'
    fpga.progdev('')
    time.sleep(.1)
    print '   Programming %s with bitstream %s' %(roach,bitstream)
    fpga.progdev(bitstream)
    time.sleep(.1)


def get_adc_power(antenna):
    adc_levels_acc_len = 32
    adc_bits = 12

    ##clear the screen:
    #print '%c[2J'%chr(27)

    #while True:
    # move cursor home
    #overflows = inst.feng_read_of(fpga)
    fpga.write_int('adc_sw_adc_sel',antenna)
    time.sleep(.1)
    rv=fpga.read_uint('adc_sw_adc_sum_sq')
            
    pwrX = float(rv)
    rmsX = numpy.sqrt(pwrX/adc_levels_acc_len)/(2**(adc_bits-1))
    bitsX = max(numpy.log2(rmsX * (2**(adc_bits))), 0.)

    #if ant<10: print '\tADC0%i:     polX:%.5f (%2.2f bits used)     polY:%.5f (%2.2f bits used)'%(ant,rmsX,bitsX,rmsY,bitsY)
    #else: print '\tADC%i:     polX:%.5f (%2.2f bits used)     polY:%.5f (%2.2f bits used)'%(ant,rmsX,bitsX,rmsY,bitsY)
    return  bitsX

def write_ctrl_sw(ctrl='ctrl_sw'):
    fpga.write_int(ctrl,0)

def change_ctrl_sw_bits(lsb, msb, val, ctrl='ctrl_sw'):
    num_bits = msb-lsb+1
    if val > (2**num_bits - 1):
        print "%d > %d"%(val,(2**num_bits - 1))
        print 'ctrl_sw MSB:', msb
        print 'ctrl_sw LSB:', lsb
        print 'ctrl_sw Value:', val
        raise ValueError("ERROR: Attempting to write value to ctrl_sw which exceeds available bit width")
    # Create a mask which has value 0 over the bits to be changed
    mask = (2**32-1) - ((2**num_bits - 1) << lsb)
    # Remove the current value stored in the ctrl_sw bits to be changed
    ctrl_sw_value = fpga.read_uint(ctrl)
    ctrl_sw_value = ctrl_sw_value & mask
    # Insert the new value
    ctrl_sw_value = ctrl_sw_value + (val << lsb)
    # Write
    fpga.write_int(ctrl,ctrl_sw_value)

def arm_sync():
    change_ctrl_sw_bits(11,11,0)
    change_ctrl_sw_bits(11,11,1)
    #change_ctrl_sw_bits(11,11,0)

def send_sync():
    change_ctrl_sw_bits(12,12,0)
    change_ctrl_sw_bits(12,12,1)
    #change_ctrl_sw_bits(12,12,0)
    
def set_fft_shift(shift):
    change_ctrl_sw_bits(0,10,shift)

def jump_eq_amp(val):
    change_ctrl_sw_bits(20,20,int(val))

def status_flag_rst():
    change_ctrl_sw_bits(19,19,1)
    change_ctrl_sw_bits(19,19,0)

def initialise_ctrl_sw(ctrl='ctrl_sw'):
    """Initialises the control software register to zero."""
    ctrl_sw=0
    write_ctrl_sw(ctrl=ctrl)

def feng_arm():
    """Arms all F engines, records arm time in config file and issues SPEAD update. Returns the UTC time at which the system was sync'd in seconds since the Unix epoch (MCNT=0)"""
    #wait for within 100ms of a half-second, then send out the arm signal.
    ready=(int(time.time()*10)%5)==0
    while not ready:
        ready=(int(time.time()*10)%5)==0
    trig_time=time.time()
    arm_sync() #implicitally affects all FPGAs
    sync_time=trig_time
    #self.sync_arm_rst()
    #if send_sync:
    send_sync()
    #send_sync()
    # Nel vecchio veniva scritto in un file chiamato come il file di conf.
    #
    #base_dir = os.path.dirname(config_file)
    #base_name = os.path.basename(config_file)
    #pkl_file = base_dir + "/sync_" + base_name.split(".xml")[0]+".pkl"
    #pickle.dump(sync_time, open(pkl_file, "wb"))
    
    # Nel nuovo lo scrivo in un software register chiamato t_start
    #fpga.write_int('t_zero',int(trig_time))
    return int(trig_time)

def read_status(trig=True, sleeptime=3):
    if trig:
        status_flag_rst()
        time.sleep(sleeptime)
    value = fpga.read_uint('status')
    return {     'Amp EQ Overflow'               :{'val':bool(value&(1<<1)),  'default':False},
                 'FFT Overflow'                  :{'val':bool(value&(1<<2)),  'default':False},
                 'Phase EQ Overflow'             :{'val':bool(value&(1<<4)),  'default':False},
                 'Sync Gen Armed'                :{'val':bool(value&(1<<6)), 'default':False}}


def head_get_keys(head_file):
    header = []
    f_head = open(head_file)
    head_list=f_head.readlines()
    #print head_list
    for i in range(len(head_list))[2:]:
        header += [head_list[i].split('#')[0].split('\t')[:-1]]
        header[i-2][0] = int(header[i-2][0])
        header[i-2][1] = int(header[i-2][1])
    f_head.close()
    #Example [[0, 4, 'T_ZERO', 'num'], [4, 4, 'HEAD_LEN', 'num']]
    return header

def head_get_info(key,header):
    #print "Looking for key=%s in header list (len=%d)"%(key,len(header))
    for i in range(len(header)):
        if header[i][2] == key:
            break
    if header[i][2] != key:
        print 'Key Error on Header: %s'%(key)
        exit()
    return header[i]
    
def write_head(val,offset,bram_header='header'):
    #print val,offset
    #print "Writing ",struct.unpack('>B',val)[0]," in offset ", offset
    fpga.write(bram_header,val,offset)
    
def write_header(field,val,header):
    record=head_get_info(field,header)
    #print record, field, val
    if record[3] == 'num':
        #if record[1] == 1:
            #valore=struct.pack('>B',val) NON FUNZIONA, ALMENO 2 BYTE!!
        #if record[1] == 2:
            #valore=struct.pack('>H',val)
        #if record[1] == 4:
        valore=struct.pack('>I',val)
    if record[3] == 'numarr':
        valore=struct.pack('>32L',val)
    if record[3] == 'str': 
        valore=val.ljust(record[1])       
    write_head(valore,record[0])
    
def read_header(field,header):
    record=head_get_info(field,header)
    if record[3] == 'num':
        #if record[1] == 1:
            #valore=struct.unpack('>B',fpga.read('header',1,record[0]))[0]
        #if record[1] == 2:
            #valore=struct.unpack('>H',fpga.read('header',2,record[0]))[0]
        #if record[1] == 4:
        valore=struct.unpack('>I',fpga.read('header',4,record[0]))[0]
    if record[3] == 'numarr':
        valore=struct.unpack('>32L',fpga.read('header',128,record[0]))        
    if record[3] == 'str': 
        valore=fpga.read('header',record[1],record[0])       
    return valore
    
def write_base_header(base_conf,header):
    for i in range(len(base_conf)):
        #print "Writing: ", base_conf.items()[i][0], base_conf.items()[i][1],
        write_header(base_conf.items()[i][0],base_conf.items()[i][1],header)
        #print " Read: ", read_header(base_conf.items()[i][0],header)
    
#def write_polbram(val,offset,bram):
 #   fpga.write(bram,val.ljust(record[1]),offset)

if __name__ == '__main__':
    from optparse import OptionParser


    p = OptionParser()
    p.set_usage('mad_start.py <ROACH_HOSTNAME_or_IP> [options]')
    p.set_description(__doc__)
    p.add_option('-p', '--skip_prog', dest='prog_fpga',action='store_false', default=True, 
        help='Skip FPGA programming (assumes already programmed).  Default: program the FPGAs')
    p.add_option('-c', '--config', dest='config_file',default="configura.conf",
        help='Select the Configuration file')
    p.add_option('-e', '--skip_eq', dest='skip_eq',action='store_true', default=False, 
        help='Skip Default Equalization.  Default: Equalize Amp 3.5 and Phase 0 to 18 ant')

    opts, args = p.parse_args(sys.argv[1:])
    #channel = opts.channel
    prog_fpga = opts.prog_fpga
    config_file =  opts.config_file

    configura,base_conf =  mad_conf_parse.parse_settings(config_file)
    roach      = configura['roach_name']
    katcp_port = configura['katcp_port']
    adc_debug  = configura['adc_debug']
    fft_shift  = base_conf['fft_shift']
    bitstream  = configura['bitstream']

    eq_amp     = configura['amp_eq']
    eq_phase     = configura['phase_eq']

    gbe_abx_name       = base_conf['gbe-0']
    gbe_abx_dest_ip    = base_conf['gbe-0_dest_ip']
    gbe_abx_dest_port  = base_conf['gbe-0_dest_port']
    gbe_abx_pkt_len    = base_conf['gbe-0_pkt_len']
    
    gbe_beam_name      = base_conf['gbe-2']
    gbe_beam_dest_ip   = base_conf['gbe-2_dest_ip']
    gbe_beam_dest_port = base_conf['gbe-2_dest_port']
    gbe_beam_pkt_len   = base_conf['gbe-2_pkt_len']

    gbe_corr_name      = base_conf['gbe-3']
    gbe_corr_dest_ip   = base_conf['gbe-3_dest_ip']
    gbe_corr_dest_port = base_conf['gbe-3_dest_port']
    gbe_corr_pkt_len   = base_conf['gbe-3_pkt_len']

    header_file  = configura['header']
    header = head_get_keys(header_file)
    pol_h=[]
    pol_v=[]
    xcorr=[]
    
    print('\n===================================\n')
    #fpga = corr.katcp_wrapper.FpgaClient('roach0')
    print('Connecting to ROACH board named "%s"... '%roach),
    fpga = corr.katcp_wrapper.FpgaClient(roach)
    time.sleep(1)
    if fpga.is_connected():
        print 'ok'
    else:
        print 'ERROR connecting to server %s on port %i.\n'%(roach,katcp_port)
        exit_fail()

    if prog_fpga:
        prog_feng(roach,bitstream)
        initialise_ctrl_sw()

    
    print "Writing base_conf..."
    write_base_header(base_conf,header)
 
    print "Setting N ant to ",base_conf["ants"]
    fpga.write_int('n_ant',base_conf["ants"])

    print "Setting Frequency channel to ",base_conf["freq_channel"]
    fpga.write_int('chan_sel',base_conf["freq_channel"])

    MAP = [0,2,4,6,1,3,5,7,8,10,12,14,9,11,13,15,16,18,20,22,17,19,21,23,24,26,28,30,25,27,29,31]
    adc_map='----------------------------------------------------------------'
    adc_map+='----------------------------------------------------------------'
    f_pol_h = open(configura['pol_h'])
    pol_h_list=f_pol_h.readlines()
    for i in xrange(len(pol_h_list)):
        pol_h += [pol_h_list[i].split()[0].split('=')]
        pol_h[i][1] = MAP[int(pol_h[i][1])]
        adc_map = adc_map[:pol_h[i][1]*4]+pol_h[i][0]+adc_map[pol_h[i][1]*4+4:]
        #if not vecchio:
        fpga.write('pol_h',struct.pack('>I',pol_h[i][1]),i*4)
    f_pol_h.close()
    print pol_h
        
    f_pol_v = open(configura['pol_v'])
    pol_v_list=f_pol_v.readlines()
    for i in xrange(len(pol_v_list)):
        pol_v += [pol_v_list[i].split()[0].split('=')]
        pol_v[i][1] = MAP[int(pol_v[i][1])]
        adc_map = adc_map[:pol_v[i][1]*4]+pol_v[i][0]+adc_map[pol_v[i][1]*4+4:]
        #if not vecchio:
        fpga.write('pol_v',struct.pack('>I',pol_v[i][1]),i*4)
    f_pol_v.close()
    print pol_v
    write_header('adc_map',adc_map,header)
    
    baseline=''
    f_xcorr = open(configura['xcorr'])
    xcorr_list=f_xcorr.readlines()
    for i in xrange(len(xcorr_list)):
        xcorr += [xcorr_list[i].split()[0].split('_')]
        baseline += xcorr[i][0]+'_'+xcorr[i][1]+'-'
    f_xcorr.close()
    #if not vecchio:
    fpga.write_int('xnum',len(xcorr_list))
    baseline = baseline[:-1]+'*'
    write_header('baseline',baseline,header)
    write_header('baseline_num',len(xcorr_list),header)
    #print xcorr
    print pol_h
    pols_adc =  pol_h + pol_v
    pols = []
    for i in range(len(pols_adc)):
        #print pols_adc[i][0]
        pols += [pols_adc[i][0]]

    #if not vecchio:
    a_factor = ""
    b_factor = ""
    #print pols_adc
    for i in range(len(xcorr)):
        a_factor += struct.pack('>I',pols_adc[pols.index(xcorr[i][0])][1])
        b_factor += struct.pack('>I',pols_adc[pols.index(xcorr[i][1])][1])
    fpga.write('A_Factor',a_factor,0)
    time.sleep(.2)
    fpga.write('B_Factor',b_factor,0)
    time.sleep(.2)
    
    print "\n HPOL\tANT-F\tVPOL\tANT-F\n-----------------------------"    
    for i in xrange(len(pol_h_list)):
        print " "+pol_h[i][0]+"\t  "+str(pol_h[i][1])+"\t"+pol_v[i][0]+"\t  "+str(pol_v[i][1])
        
    print "\n\n Correlation List (A * Bconj)\n----------------------------------"    
    for i in range(len(xcorr_list))[::2]:
        print " "+xcorr[i][0]+" <---> "+xcorr[i][1]+"    "+xcorr[i+1][0]+" <---> "+xcorr[i+1][1]
        

    print "\nStarting interface %s"%(gbe_abx_name)
    fpga.write_int(gbe_abx_name+'_destip',gbe_abx_dest_ip)
    time.sleep(0.3)
    fpga.write_int(gbe_abx_name+'_destport',gbe_abx_dest_port)
    time.sleep(0.3)
    fpga.write_int(gbe_abx_name+'_len',gbe_abx_pkt_len)
    time.sleep(0.3)
    ipconv = str(int((gbe_abx_dest_ip & 255*256*256*256) >> 24))
    ipconv += "."+str(int((gbe_abx_dest_ip & 255*256*256) >> 16))
    ipconv += "."+str(int((gbe_abx_dest_ip & 255*256) >> 8))
    ipconv += "."+str(int(gbe_abx_dest_ip & 255))
    print "Set UDP packets destination IP:Port to %s:%d"%(ipconv,gbe_abx_dest_port)
    print "Set UDP packets size to %d (64 bit + 1 counter)"%(gbe_abx_pkt_len)
    ip = 3232238524
    mac = (0<<40) + (96<<32) + ip
    fpga.tap_start('tap0', gbe_abx_name, mac, ip, gbe_abx_dest_port) 
    time.sleep(0.3)
    print "UDP packets started!"
    #print('\n===================================\n')

    print "\nStarting interface %s"%(gbe_beam_name)
    fpga.write_int('gbe_beam_ip',gbe_beam_dest_ip)
    time.sleep(0.3)
    fpga.write_int('gbe_beam_port',gbe_beam_dest_port)
    time.sleep(0.3)
    fpga.write_int(gbe_beam_name+'_len',gbe_beam_pkt_len)
    time.sleep(0.3)
    ipconv = str(int((gbe_beam_dest_ip & 255*256*256*256) >> 24))
    ipconv += "."+str(int((gbe_beam_dest_ip & 255*256*256) >> 16))
    ipconv += "."+str(int((gbe_beam_dest_ip & 255*256) >> 8))
    ipconv += "."+str(int(gbe_beam_dest_ip & 255))
    print "Set UDP packets destination IP:Port to %s:%d"%(ipconv,gbe_beam_dest_port)
    print "Set UDP packets size to %d (64 bit + 1 counter)"%(gbe_beam_pkt_len)
    ip = 3232238525
    mac = (0<<40) + (96<<32) + ip
    fpga.tap_start('tap2', gbe_beam_name, mac, ip, gbe_beam_dest_port) 
    time.sleep(0.3)
    print "UDP packets started!"

    print "\nStarting interface %s"%(gbe_corr_name)
    fpga.write_int('gbe_corr_ip',gbe_corr_dest_ip)
    time.sleep(0.3)
    fpga.write_int('gbe_corr_port',gbe_corr_dest_port)
    time.sleep(0.3)
    fpga.write_int(gbe_corr_name+'_len',gbe_corr_pkt_len)
    time.sleep(0.3)
    ipconv = str(int((gbe_corr_dest_ip & 255*256*256*256) >> 24))
    ipconv += "."+str(int((gbe_corr_dest_ip & 255*256*256) >> 16))
    ipconv += "."+str(int((gbe_corr_dest_ip & 255*256) >> 8))
    ipconv += "."+str(int(gbe_corr_dest_ip & 255))
    print "Set UDP packets destination IP:Port to %s:%d"%(ipconv,gbe_corr_dest_port)
    print "Set UDP packets size to %d (64 bit + 1 counter)"%(gbe_corr_pkt_len)
    ip = 3232238526
    mac = (0<<40) + (96<<32) + ip
    fpga.tap_start('tap3', gbe_corr_name, mac, ip, gbe_corr_dest_port) 
    time.sleep(0.3)
    print "UDP packets started!\n"
    #print('\n===================================\n')

    time.sleep(0.1)
    set_fft_shift(fft_shift)
    
    jump_eq_amp_flag=True
    if jump_eq_amp_flag:
        print "EQ AMP FLAG: ",jump_eq_amp_flag,", it means the amplitude equalization block is active!"
    else:
        print "EQ AMP FLAG: ",jump_eq_amp_flag,", it means the amplitude equalization block is skipped!"
    jump_eq_amp(jump_eq_amp_flag)
    
    print   '\n   Calibrating ADC on %s' %(roach)
    adc_cal();
    time.sleep(0.05)
    fpga.write_int('adc_spi_ctrl', 1)
    time.sleep(.05)
    fpga.write_int('adc_spi_ctrl', 0)
    time.sleep(.05)
    time.sleep(0.5)
    check_adc_sync()
    time.sleep(0.5)
    check_adc_sync()
    time.sleep(0.5)
    check_adc_sync()
    time.sleep(0.5)
    check_adc_sync()
    time.sleep(0.5)
    check_adc_sync()

    # ARM THE FENGINE
    print ''' Arming F Engine and setting FFT Shift...''',
    sys.stdout.flush()
    trig_time=feng_arm()
    print ' Armed.\nExpect trigger at %s local (%s UTC).'%(time.strftime('%H:%M:%S',time.localtime(trig_time)),time.strftime('%H:%M:%S',time.gmtime(trig_time))), 
    print "Updating header BRAM with %s=%d"%('t_zero',trig_time)
    write_header('t_zero',trig_time,header)
    print "Read from header t_zero=%d"%(read_header('t_zero',header))
    print "Updating header BRAM with %s=%d"%('fft_shift',fft_shift)
    write_header('fft_shift',fft_shift,header)
    print "Read from header fft_shift=%d"%(read_header('fft_shift',header))
    #Control Settings
    sync_arm = False
    sync_rst = False

    arm_sync()

    if not opts.skip_eq:
        #os.system('./mad_eq.py -A eq/default_amplitude_correction.txt -P eq/0_phase_correction.txt')
        os.system('./mad_eq.py -A eq/'+eq_amp+' -P eq/'+eq_phase)




