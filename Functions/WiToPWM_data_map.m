function output = WiToPWM_data_map(low1, high1, low2, high2, single_data)

    pente = (high2 - low2)/(high1 - low1) ;
    offset = - pente*low1 ;


    output = pente*single_data + offset ;
end

