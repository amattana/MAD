import ConfigParser

def parse_settings(filename):
    conf = ConfigParser.SafeConfigParser()
    configura={}
    base_conf={}
    if not [filename] == conf.read(filename):
        print conf.read(filename)
        print "Cannot parse file " + filename
        return configura,base_conf
    conf.optionxform = str
    configura["pol_h"] = conf.get("CONFIG_FILE", "pol_h")             
    configura["pol_v"] = conf.get("CONFIG_FILE", "pol_v")             
    configura["xcorr"] = conf.get("CONFIG_FILE", "xcorr")
    configura["amp_eq"] = conf.get("CONFIG_FILE", "amp_eq")
    configura["phase_eq"] = conf.get("CONFIG_FILE", "phase_eq")
    configura["rx_network"] = conf.get("CONFIG_FILE", "rx_network")             
    configura["rx_map"] = conf.get("CONFIG_FILE", "rx_map")
    
    configura["adc_curve"] = conf.get("CONFIG_FILE", "adc_curve")
    configura["header"] = conf.get("CONFIG_FILE", "header")
    configura["bitstream"] = conf.get("FENG_CONF", "bitstream")            
    configura["adc_debug"] = conf.get("FENG_CONF", "adc_debug")            
    #configura["fft_shift"] = int(conf.get("FENG_CONF", "fft_shift"))            
    configura["roach_name"] = conf.get("FENG_CONF", "roach_name")            
    configura["katcp_port"] = int(conf.get("FENG_CONF", "katcp_port"))            

    base_conf["bof_file"] = conf.get("FENG_CONF", "bitstream") 
    #base_conf["pkt_len"] = int(conf.get("FENG_CONF", "pkt_len")) 
    base_conf["pkt_len"] = int(conf.get("FENG_CONF", "pkt_len")) 
    base_conf["head_len"] = int(conf.get("FENG_CONF", "head_len")) 
                
    base_conf["adc_name"] = conf.get("FENG_CONF", "adc_name")   
    base_conf["clock_rate"] = int(conf.get("FENG_CONF", "clock_rate"))   
    base_conf["sample_rate"] = int(conf.get("FENG_CONF", "sample_rate"))  
    base_conf["pfb_size"] = int(conf.get("FENG_CONF", "pfb_size"))         
    base_conf["pfb_in_signals"] = int(conf.get("FENG_CONF", "pfb_in_signals"))  
    base_conf["pfb_window"] = conf.get("FENG_CONF", "pfb_window")        
    base_conf["pfb_in_bitwidth"] = int(conf.get("FENG_CONF", "pfb_in_bitwidth"))   
    base_conf["pfb_out_bitwidth"] = int(conf.get("FENG_CONF", "pfb_out_bitwidth")) 
    base_conf["pfb_coeff_bitwidth"] = int(conf.get("FENG_CONF", "pfb_coeff_bitwidth")) 
    base_conf["pfb_quant_behavior"] = conf.get("FENG_CONF", "pfb_quant_behavior") 
    base_conf["fft_size"] = int(conf.get("FENG_CONF", "fft_size"))      
    base_conf["fft_shift"] = int(conf.get("FENG_CONF", "fft_shift"))      
    base_conf["fft_in_bitwidth"] = int(conf.get("FENG_CONF", "fft_in_bitwidth")) 
    base_conf["fft_out_bitwidth"] = int(conf.get("FENG_CONF", "fft_out_bitwidth"))  
    base_conf["fft_quant_behavior"] = conf.get("FENG_CONF", "fft_quant_behavior")
    base_conf["fft_of_behavior"] = conf.get("FENG_CONF", "fft_of_behavior")

    base_conf["observ_site"] = conf.get("OBSERVATION", "observ_site")
    base_conf["antenna_type"] = conf.get("OBSERVATION", "antenna_type")
    base_conf["ants"] = int(conf.get("OBSERVATION", "ants"))
    base_conf["pols"] = int(conf.get("OBSERVATION", "pols"))
    base_conf["freq_channel"] = int(conf.get("OBSERVATION", "freq_channel"))

    base_conf["gbe-0"] = conf.get("FENG_CONF", "gbe-0")            
    base_conf["gbe-0_dest_ip"] = int(conf.get("FENG_CONF", "gbe-0_dest_ip"))            
    base_conf["gbe-0_dest_port"] = int(conf.get("FENG_CONF", "gbe-0_dest_port"))
    base_conf["gbe-0_pkt_len"] = int(conf.get("FENG_CONF", "gbe-0_pkt_len"))
    base_conf["gbe-1"] = conf.get("FENG_CONF", "gbe-1")            
    base_conf["gbe-1_dest_ip"] = int(conf.get("FENG_CONF", "gbe-1_dest_ip"))            
    base_conf["gbe-1_dest_port"] = int(conf.get("FENG_CONF", "gbe-1_dest_port"))
    base_conf["gbe-1_pkt_len"] = int(conf.get("FENG_CONF", "gbe-1_pkt_len"))
    base_conf["gbe-2"] = conf.get("FENG_CONF", "gbe-2")            
    base_conf["gbe-2_dest_ip"] = int(conf.get("FENG_CONF", "gbe-2_dest_ip"))            
    base_conf["gbe-2_dest_port"] = int(conf.get("FENG_CONF", "gbe-2_dest_port"))
    base_conf["gbe-2_pkt_len"] = int(conf.get("FENG_CONF", "gbe-2_pkt_len"))
    base_conf["gbe-3"] = conf.get("FENG_CONF", "gbe-3")            
    base_conf["gbe-3_dest_ip"] = int(conf.get("FENG_CONF", "gbe-3_dest_ip"))            
    base_conf["gbe-3_dest_port"] = int(conf.get("FENG_CONF", "gbe-3_dest_port"))
    base_conf["gbe-3_pkt_len"] = int(conf.get("FENG_CONF", "gbe-3_pkt_len"))

    return configura,base_conf
    
