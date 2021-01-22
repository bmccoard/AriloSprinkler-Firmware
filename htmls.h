const char ap_home_html[] PROGMEM = R"(<head>
<title>OpenSprinkler WiFi Config</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
</head>
<body>
<style> table, th, td {	border: 0px solid black;  border-collapse: collapse;}
table#rd th { border: 1px solid black;}
table#rd td {	border: 1px solid black; border-collapse: collapse;}</style>
<caption><b>OpenSprinkler WiFi Config</caption><br><br>
<table cellspacing=4 id='rd'>
<tr><td>Detected SSIDs</td><td>Strength</td><td>Power Level</td></tr>
<tr><td>(Scanning...)</td></tr>
</table>
<br><br>
<table cellspacing=16>
<tr><td><input type='text' name='ssid' id='ssid' style='font-size:14pt;height:28px;'></td><td>(Your WiFi SSID)</td></tr>
<tr><td><input type='password' name='pass' id='pass' style='font-size:14pt;height:28px;'></td><td>(Your WiFi Password)</td></tr>
<tr><td><input type='text' name='auth' id='auth' style='font-size:14pt;height:28px;'></td><td><label id='lbl_auth'>(Blynk Token, Optional)</label></td></tr>
<tr><td colspan=2><p id='msg'></p></td></tr>
<tr><td><button type='button' id='butt' onclick='sf();' style='height:36px;width:180px'>Submit</button></td><td></td></tr>
</table>
<script>
function id(s) {return document.getElementById(s);}
function sel(i) {id('ssid').value=id('rd'+i).value;}
var tci;
function tryConnect() {
var xhr=new XMLHttpRequest();
xhr.onreadystatechange=function() {
if(xhr.readyState==4 && xhr.status==200) {
var jd=JSON.parse(xhr.responseText);
if(jd.ip==0) return;
var ip=''+(jd.ip%256)+'.'+((jd.ip/256>>0)%256)+'.'+(((jd.ip/256>>0)/256>>0)%256)+'.'+(((jd.ip/256>>0)/256>>0)/256>>0);
id('msg').innerHTML='<b><font color=green>Connected! Device IP: '+ip+'</font></b><br>Device is rebooting. Switch back to<br>the above WiFi network, and then<br>click the button below to redirect.'
id('butt').innerHTML='Go to '+ip;id('butt').disabled=false;
id('butt').onclick=function rd(){window.open('http://'+ip);}
clearInterval(tci);
}
}    
xhr.open('GET', 'jtap', true); xhr.send();
}  
function sf() {
id('msg').innerHTML='';
var xhr=new XMLHttpRequest();
xhr.onreadystatechange=function() {
if(xhr.readyState==4 && xhr.status==200) {
var jd=JSON.parse(xhr.responseText);
if(jd.result==1) { id('butt').innerHTML='Connecting...'; id('msg').innerHTML='<font color=gray>Connecting, please wait...</font>'; tci=setInterval(tryConnect, 1000); return; }
id('msg').innerHTML='<b><font color=red>Error code: '+jd.result+', item: '+jd.item+'</font></b>'; id('butt').innerHTML='Submit'; id('butt').disabled=false;id('ssid').disabled=false;id('pass').disabled=false;id('auth').disabled=false;
}
};
var comm='ccap?ssid='+encodeURIComponent(id('ssid').value)+'&pass='+encodeURIComponent(id('pass').value)+'&auth='+id('auth').value;
xhr.open('GET', comm, true); xhr.send();
id('butt').disabled=true;id('ssid').disabled=true;id('pass').disabled=true;id('auth').disabled=true;
}

function loadSSIDs() {
var xhr=new XMLHttpRequest();
xhr.onreadystatechange=function() {
if(xhr.readyState==4 && xhr.status==200) {
id('rd').deleteRow(1);
var i, jd=JSON.parse(xhr.responseText);
for(i=0;i<jd.ssids.length;i++) {
var signalstrength= jd.rssis[i]>-71?'Ok':(jd.rssis[i]>-81?'Weak':'Poor');
var row=id('rd').insertRow(-1);
row.innerHTML ="<tr><td><input name='ssids' id='rd"+i+"' onclick='sel(" + i + ")' type='radio' value='"+jd.ssids[i]+"'>" + jd.ssids[i] + "</td>"  + "<td align='center'>"+signalstrength+"</td>" + "<td align='center'>("+jd.rssis[i] +" dbm)</td>" + "</tr>";
}
};
}
xhr.open('GET','jsap',true); xhr.send();
}
setTimeout(loadSSIDs, 1000);
</script>
</body>
)";
const char ap_update_html[] PROGMEM = R"(<head>
<title>OpenSprinkler Firmware Update</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
</head>
<body>
<div id='page_update'>
<div><h3>OpenSprinkler AP-mode Firmware Update</h3></div>
<div>
<form method='POST' action='/update' id='fm' enctype='multipart/form-data'>
<table cellspacing=4>
<tr><td><input type='file' name='file' accept='.bin' id='file'></td></tr>
<tr><td><b>Device password: </b><input type='password' name='pw' size=36 maxlength=36 id='pw'></td></tr>
<tr><td><label id='msg'></label></td></tr>
</table>
<button id='btn_submit' style='height:48px;'>Submit</a>
</form>
</div>
</div>
<script>
function id(s) {return document.getElementById(s);}
function clear_msg() {id('msg').innerHTML='';}
function show_msg(s,t,c) {
id('msg').innerHTML=s.fontcolor(c);
if(t>0) setTimeout(clear_msg, t);
}
id('btn_submit').addEventListener('click', function(e){
e.preventDefault();
var files= id('file').files;
if(files.length==0) {show_msg('Please select a file.',2000,'red'); return;}
if(id('pw').value=='') {
if(!confirm('You did not input a device key. Are you sure?')) return;
}
show_msg('Uploading. Please wait...',10000,'green');
var fd = new FormData();
var file = files[0];
fd.append('file', file, file.name);
fd.append('pw', id('pw').value);
var xhr = new XMLHttpRequest();
xhr.onreadystatechange = function() {
if(xhr.readyState==4 && xhr.status==200) {
var jd=JSON.parse(xhr.responseText);
if(jd.result==1) {
show_msg('Update is successful. Rebooting.',0,'green');
id('pw').value='';
} else if (jd.result==2) {
show_msg('Check device key and try again.', 10000, 'red');
} else {
show_msg('Update failed.',0,'red');
}
}
};
xhr.open('POST', 'update', true);
xhr.send(fd);
});
</script>
</body>
)";
const char mirrorlink_control_html[] PROGMEM = R"(<head>
<title>AriloSprinkler MirrorLink Control</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
</head>
<body style='background-color: rgba(195, 247, 208, 0.473)'>
<h1 style='color:white;background-color:black;width:650px;text-align:center'>AriloSprinkler MirrorLink Control Panel</h1>
<style> table, th, td { border: 0px solid black; border-collapse: collapse; padding: 5px;}
table#mlsg th { border: 3px solid black;}
table#mlsg td { border: 3px solid black; border-collapse: collapse;}</style>
<caption><b style='color:black'>General Status</caption><br><br>
<table cellspacing=4 id='mlsg'>
<tr><td>Mode&nbsp&nbsp</td><td>NetworkID&nbsp&nbsp</td></tr>
<tr><td>(Waiting...)</td></tr>
</table>
<br><br>
<style> table, th, td { border: 0px solid black; border-collapse: collapse; padding: 5px;}
table#mlsr th { border: 3px solid black;}
table#mlsr td { border: 3px solid black; border-collapse: collapse;}</style>
<caption><b style='color:black'>Radio Status</caption><br><br>
<table cellspacing=4 id='mlsr'>
<tr><td>Frequency&nbsp&nbsp</td><td>DutyCycle&nbsp&nbsp</td><td>TX Power&nbsp&nbsp</td><td>Local RSSI&nbsp&nbsp</td><td>Remote RSSI&nbsp&nbsp</td><td>Local SNR&nbsp&nbsp</td><td>Remote SNR&nbsp&nbsp</td><td>Association Status&nbsp&nbsp</td><td>Association Attempts&nbsp&nbsp</td></tr>
<tr><td>(Waiting...)</td></tr>
</table>
<br><br>
<style> table, th, td { border: 0px solid black;  border-collapse: collapse; padding: 5px;}
table#mlsp th { border: 3px solid black;}
table#mlsp td { border: 3px solid black; border-collapse: collapse;}</style>
<caption><b style='color:black'>Packet Status</caption><br><br>
<table cellspacing=4 id='mlsp'>
<tr><td>Buffered Packets&nbsp&nbsp</td><td>Packets Sent&nbsp&nbsp</td><td>Packets Received&nbsp&nbsp</td><td>Encryption&nbsp&nbsp</td><td>Packet Sent Time&nbsp&nbsp</td><td>No TX Time&nbsp&nbsp</td>
<tr><td>(Waiting...)</td></tr>
</table>
<br><br>
<table cellspacing=16>
<tr><td><input type='number' name='netid' id='netid' min='0' max='255' style='font-size:12pt;height:28px;width:120px;'></td><td>Network ID (0 to 255)</td></tr>
<tr><td><input type='number' name='mlpass1' id='mlpass1' min='0' max='4294967295' style='font-size:12pt;height:28px;width:120px;'></td><td>Association Key 1 (32bit max)</td></tr>
<tr><td><input type='number' name='mlpass2' id='mlpass2' min='0' max='4294967295' style='font-size:12pt;height:28px;width:120px'></td><td>Association Key 2 (32bit max)</td></tr>
<tr><td><input type='number' name='mlpass3' id='mlpass3' min='0' max='4294967295' style='font-size:12pt;height:28px;width:120px'></td><td>Association Key 3 (32bit max)</td></tr>
<tr><td><input type='number' name='mlpass4' id='mlpass4' min='0' max='4294967295' style='font-size:12pt;height:28px;width:120px'></td><td>Association Key 4 (32bit max)</td></tr>
<tr><td><input type='number' name='mlchan' id='mlchan' min='0' max='5' style='font-size:14pt;height:28px;'></td><td>Channel (0 to 5)</td></tr>
<tr><td><input type='checkbox' name='mlfhop' id='mlfhop' value='1' style='font-size:14pt;height:28px;'></td><td>Frequency Hopping</td></tr>
<tr><td><input type='number' name='mlplim' id='mlplim' min='-20' max='30' style='font-size:14pt;height:28px;'></td><td>Power Limit (-20 to +30dBm)</td></tr>
<tr><td><input type='checkbox' name='mlatpc' id='mlatpc' value='1' style='font-size:14pt;height:28px;'></td><td>ATPC (Adaptive Transmission Power Control)</td></tr>
<tr><td><input type='number' name='dtcycl' id='dtcycl' min='0.0' max='100.0' step='0.1' style='font-size:14pt;height:28px;'></td><td>Duty Cycle (0.0 to 100.0)</td></tr>
<tr><td><input type='checkbox' name='mlrem' id='mlrem' value='1' style='font-size:14pt;height:28px;'></td><td>Remote Mode</td></tr>
<tr><td colspan=2><p id='msg'></p></td></tr>
<tr><td><button type='button' id='butt' onclick='mlc();' style='height:36px;width:130px'>Submit</button></td><td></td></tr>
</table>
<script>
function id(s) {return document.getElementById(s);}
function mlc() {
id('msg').innerHTML='';
var xhr=new XMLHttpRequest();
xhr.onreadystatechange=function() {
if(xhr.readyState==4 && xhr.status==200) {
var jd=JSON.parse(xhr.responseText);
if(jd.result==1) { return; }
id('msg').innerHTML='<b><font color=red>Error code: '+jd.result+', item: '+jd.item+'</font></b>'; id('butt').innerHTML='Submit'; id('butt').disabled=false;id('mlpass1').disabled=false;id('mlpass2').disabled=false;id('mlpass3').disabled=false;id('mlpass4').disabled=false;id('mlchan').disabled=false;id('mlplim').disabled=false;id('dtcycl').disabled=false;id('mlrem').disabled=false;
}
};
var checkrem=0;
var checkfhop=0;
var checkatpc=0;
if (id('mlrem').checked == true) { checkrem = 1; }
if (id('mlfhop').checked == true) { checkfhop = 1; }
if (id('mlatpc').checked == true) { checkatpc = 1; }
var comm='mlchconfig?netid='+encodeURIComponent(id('netid').value)+'&mlpass1='+encodeURIComponent(id('mlpass1').value)+'&mlpass2='+encodeURIComponent(id('mlpass2').value)+'&mlpass3='+encodeURIComponent(id('mlpass3').value)+'&mlpass4='+encodeURIComponent(id('mlpass4').value)+'&mlchan='+encodeURIComponent(id('mlchan').value)+'&mlfhop='+checkfhop+'&mlplim='+encodeURIComponent(id('mlplim').value)+'&mlatpc='+checkatpc+'&dtcycl='+encodeURIComponent(id('dtcycl').value)+'&mlrem='+checkrem;
xhr.open('GET', comm, true); xhr.send();
}
function showStatusGeneral() {
var xhr=new XMLHttpRequest();
xhr.onreadystatechange=function() {
if(xhr.readyState==4 && xhr.status==200) {
id('mlsg').deleteRow(1);
var jd=JSON.parse(xhr.responseText);
var row=id('mlsg').insertRow(-1);
row.innerHTML ="<tr><td align='center'>("+jd.mode +")</td>" + "<td align='center'>("+jd.networkid+")</td>" + "</tr>";
};
}
xhr.open('GET','mlstatusgeneral',true); xhr.send();
}
setInterval(showStatusGeneral, 10000);
function showStatusRadio() {
var xhr=new XMLHttpRequest();
xhr.onreadystatechange=function() {
if(xhr.readyState==4 && xhr.status==200) {
id('mlsr').deleteRow(1);
var jd=JSON.parse(xhr.responseText);
var row=id('mlsr').insertRow(-1);
row.innerHTML ="<tr><td align='center'>("+jd.frequency+" MHz)</td>" + "<td align='center'>("+jd.dutycycle+"%)</td>" + "<td align='center'>("+jd.powerlevel+" dbm)</td>" + "<td align='center'>("+jd.rssis[0]+" dbm)</td>" + "<td align='center'>("+jd.rssis[1] +" dbm)</td>" + "<td align='center'>("+jd.snrs[0] +" db)</td>" + "<td align='center'>("+jd.snrs[1] +" db)</td>" + "<td align='center'>("+jd.assocst +")</td>" + "<td align='center'>("+jd.assocatm +")</td>" + "</tr>";
};
}
xhr.open('GET','mlstatusradio',true); xhr.send();
}
setInterval(showStatusRadio, 5000);
function showStatusPackets() {
var xhr=new XMLHttpRequest();
xhr.onreadystatechange=function() {
if(xhr.readyState==4 && xhr.status==200) {
id('mlsp').deleteRow(1);
jd=JSON.parse(xhr.responseText);
row=id('mlsp').insertRow(-1);
row.innerHTML ="<tr><td align='center'>("+jd.buffpackets+")</td>"  + "<td align='center'>("+jd.packetstx+")</td>" + "<td align='center'>("+jd.packetsrx +")</td>" + "<td align='center'>("+jd.encryption +")</td>" + "<td align='center'>("+jd.packettime +" msec)</td>" + "<td align='center'>("+jd.notxtime +" sec)</td>" + "</tr>";
};
}
xhr.open('GET','mlstatuspackets',true); xhr.send();
}
setInterval(showStatusPackets, 7000);
</script>
</body>
)";
const char sta_update_html[] PROGMEM = R"(<head>
<title>OpenSprinkler Firmware Update</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<link rel='stylesheet' href='http://code.jquery.com/mobile/1.3.1/jquery.mobile-1.3.1.min.css' type='text/css'>
<script src='http://code.jquery.com/jquery-1.9.1.min.js' type='text/javascript'></script>
<script src='http://code.jquery.com/mobile/1.3.1/jquery.mobile-1.3.1.min.js' type='text/javascript'></script>
</head>
<body>
<div data-role='page' id='page_update'>
<div data-role='header'><h3>OpenSprinkler Firmware Update</h3></div>
<div data-role='content'>
<form method='POST' action='/update' id='fm' enctype='multipart/form-data'>
<table cellspacing=4>
<tr><td><input type='file' name='file' accept='.bin' id='file'></td></tr>
<tr><td><b>Device password: </b><input type='password' name='pw' size=36 maxlength=36 id='pw'></td></tr>
<tr><td><label id='msg'></label></td></tr>
</table>
<a href='#' data-role='button' data-inline='true' data-theme='b' id='btn_submit'>Submit</a>
</form>
</div>
<div data-role='footer' data-theme='c'>
<p style='font-weight:normal;'>&copy; OpenSprinkler (<a href='http://www.opensprinkler.com' target='_blank' style='text-decoration:none'>www.opensprinkler.com</a>)</p>
</div>  
</div>
<script>
function id(s) {return document.getElementById(s);}
function clear_msg() {id('msg').innerHTML='';}
function show_msg(s,t,c) {
id('msg').innerHTML=s.fontcolor(c);
if(t>0) setTimeout(clear_msg, t);
}
$('#btn_submit').click(function(e){
var files= id('file').files;
if(files.length==0) {show_msg('Please select a file.',2000,'red'); return;}
if(id('pw').value=='') {
if(!confirm('You did not input a device password. Are you sure?')) return;
}else{beforeSubmit();}
show_msg('Uploading. Please wait...',0,'green');
var fd = new FormData();
var file = files[0];
fd.append('file', file, file.name);
fd.append('pw', id('pw').value);
var xhr = new XMLHttpRequest();
xhr.onreadystatechange = function() {
if(xhr.readyState==4 && xhr.status==200) {
var jd=JSON.parse(xhr.responseText);
if(jd.result==1) {
show_msg('Update is successful. Rebooting.',10000,'green');
id('pw').value='';
} else if (jd.result==2) {
show_msg('Check device password and try again.', 10000, 'red');
} else {
show_msg('Update failed.',0,'red');
}
}
};
xhr.open('POST', 'update', true);
xhr.send(fd);
});
</script>
<script src=https://ui.opensprinkler.com/js/hasher.js></script>
</body>
)";
