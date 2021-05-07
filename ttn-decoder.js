// Decoder script for The Things Network v3

// GPS Tracker Decoder
// See https://github.com/radforschung/Lora-TTNMapper-T-Beam#instructions

function decodeGPS (bytes) {
    var decoded = {};

    decoded.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;
  
    decoded.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;
  
    var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
    var sign = bytes[6] & (1 << 7);
    if(sign)
    {
        decoded.altitude = 0xFFFF0000 | altValue;
    }
    else
    {
        decoded.altitude = altValue;
    }
  
    decoded.hdop = bytes[8] / 10.0;

    return decoded;
}

// Wifi Tracker Decoder
// See https://github.com/stadtulm/Lora-Wifi-Location-Tracker/blob/master/ttn-decoder-script.js

function decodeWifi (bytes) { 
    var devices = [];
    for (var i = 0; i * 6 < bytes.length; i++) {
        addrbytes = bytes.slice(i * 6, i * 6 + 6);
        var addrArray = [];
        for (var j = 0; j < 6; j++) {
            var str = addrbytes[j].toString(16);
            if (str.length == 1) {
                str = '0' + str;
            }
            addrArray.push(str);
        }
        devices.push({
            macAddress: addrArray.join(":")
        });
    }

    var decoded = {
        "wifiAccessPoints": devices,
        "fallbacks": {
            "lacf": false,
            "ipf": false
        }
    };

    return JSON.stringify(decoded)
}

// Main Decoder

function decodeUplink(input) {
    
    var data = {};
    var warnings = {};
    var errors = {};

    switch(input.bytes[0]) {
        case 0x10:
            data = decodeGPS(bytes.slice(3, 11 + 1));
            break;
        case 0x11:
            data = decodeGPS(bytes.slice(3, 11 + 1));
            data.wifi = decodeWifi(bytes.slice(12, bytes.length))
            break;
        case 0x12:
            data.wifi = decodeWifi(bytes.slice(3, bytes.length))
            break;
        case 0x01:
            warnings = ["Position could not be determined"]
            break;
        default:
            errors = ["Invalid payload format"]
    }

    data.voltage = ((bytes[1]<<8) + bytes[2]) / 100;

    return {
        data,
        warnings,
        errors
    }
}