/**
 * ESP32 Control JS
 */
var fanStateCache = null;
var seconds = null;
var otaTimerVar = null;
var wifiConnectInterval = null;
var scheduleFormDirty = false;
var fanFormDirty = false;
var fanDirtyUntil = 0;
var tempHistory = [];
const TEMP_HISTORY_MAX = 60;

function markFanDirty(extraMs = 30000) {
    fanFormDirty = true;
    fanDirtyUntil = Date.now() + extraMs;
}

function pushTempSample(val){
    if (isNaN(val)) return;
    tempHistory.push({ t: Date.now(), v: val });
    if (tempHistory.length > TEMP_HISTORY_MAX) tempHistory.shift();
    drawTempChart();
}

function drawTempChart(){
    var canvas = document.getElementById('temp_chart');
    if(!canvas) return;
    var ctx = canvas.getContext('2d');
    var w = canvas.width;
    var h = canvas.height;
    ctx.clearRect(0,0,w,h);

    // background
    var grd = ctx.createLinearGradient(0,0,0,h);
    grd.addColorStop(0,'rgba(14,165,233,0.06)');
    grd.addColorStop(1,'rgba(14,165,233,0.0)');
    ctx.fillStyle = grd;
    ctx.fillRect(0,0,w,h);

    if(tempHistory.length < 2){
        ctx.fillStyle = '#9ca3af';
        ctx.font = '13px Segoe UI, sans-serif';
        ctx.fillText('Esperando datos...', 12, h/2);
        return;
    }

    var vals = tempHistory.map(function(p){ return p.v; });
    var min = Math.min.apply(null, vals);
    var max = Math.max.apply(null, vals);
    if (max - min < 0.5) { max = min + 0.5; }
    var pad = 12;
    function scaleX(i){
        if (tempHistory.length === 1) return pad;
        return pad + (w - pad*2) * (i / (tempHistory.length - 1));
    }
    function scaleY(v){
        return (h - pad) - ((v - min) / (max - min)) * (h - pad*2);
    }

    ctx.lineWidth = 2;
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)';
    ctx.beginPath();
    ctx.moveTo(pad, scaleY(min));
    ctx.lineTo(w - pad, scaleY(min));
    ctx.stroke();

    ctx.strokeStyle = '#0ea5e9';
    ctx.beginPath();
    tempHistory.forEach(function(p, idx){
        var x = scaleX(idx);
        var y = scaleY(p.v);
        if(idx === 0) ctx.moveTo(x,y);
        else ctx.lineTo(x,y);
    });
    ctx.stroke();

    ctx.fillStyle = '#22c55e';
    tempHistory.forEach(function(p, idx){
        var x = scaleX(idx);
        var y = scaleY(p.v);
        ctx.beginPath();
        ctx.arc(x,y,2.5,0,Math.PI*2);
        ctx.fill();
    });
}

// Detecta conflicto de horarios solapados con días coincidentes
function schedulesConflict(idx, payload) {
    function contains(time, start, end) {
        return (start <= end) ? (time >= start && time <= end)
                              : (time >= start || time <= end); // rango cruzando medianoche
    }
    function overlap(start1, end1, start2, end2) {
        return contains(start1, start2, end2) || contains(start2, start1, end1);
    }

    var pStart = payload.start_hour * 60 + payload.start_minute;
    var pEnd   = payload.end_hour   * 60 + payload.end_minute;

    if (!fanStateCache || !fanStateCache.schedules) return false;
    for (var i = 0; i < fanStateCache.schedules.length; i++) {
        if (i === idx) continue;
        var s = fanStateCache.schedules[i];
        if (!s.active) continue;
        var sStart = s.start_hour * 60 + s.start_minute;
        var sEnd   = s.end_hour   * 60 + s.end_minute;
        var daysOverlap = (s.days_mask & payload.days_mask) !== 0;
        if (daysOverlap && overlap(pStart, pEnd, sStart, sEnd)) return true;
    }
    return false;
}

function updateModeUI(mode) {
    var isManual = mode === 0;
    var isAuto = mode === 1;
    var isProg = mode === 2;
    var manualSlider = document.getElementById('manual_pwm_slider');
    var tmin = document.getElementById('t_min');
    var tmax = document.getElementById('t_max');
    var progBlock = document.getElementById('program_registers');

    manualSlider.disabled = !isManual;
    tmin.disabled = !isAuto;
    tmax.disabled = !isAuto;
    progBlock.style.display = isProg ? 'block' : 'none';
}

$(document).ready(function(){
    startFanPolling();
    startLogsPolling();
    
    $("#connect_wifi").on("click", function(){
        checkCredentials();
    });
    $("#manual_pwm_slider").on("input", function(){
        $("#manual_pwm_label").text(this.value + "%");
        markFanDirty();
    });
    $("#manual_pwm_slider").on("mousedown touchstart", function(){
        markFanDirty();
    });
    $("input[name='fan_mode']").on("change", function(){
        var mode = parseInt($("input[name='fan_mode']:checked").val());
        updateModeUI(mode);
        markFanDirty();
        applyFanConfig();
    });
    $("#t_min, #t_max").on("input change", function(){
        markFanDirty();
    });
    $("#selectNumber").on("change", function(){
        scheduleFormDirty = false;
        populateScheduleForm();
    });
    $("#start_time, #end_time, #temp0, #temp100, #sched_active").on("input change", function(){
        scheduleFormDirty = true;
    });
    for (var i = 0; i < 7; i++) {
        $("#day_" + i).on("change", function(){
            scheduleFormDirty = true;
        });
    }
    for (let i = 0; i < 3; i++) {
        $("#reg_prog_" + i).on("change", function(){
            toggleProgramRegister(i, $(this).is(":checked"));
        });
    }
});

/** OTA **/
function getFileInfo() 
{
    var x = document.getElementById("selected_file");
    var file = x.files[0];
    document.getElementById("file_info").innerHTML = "<h4>File: " + file.name + "<br>" + "Size: " + file.size + " bytes</h4>";
}

function updateFirmware() 
{
    var formData = new FormData();
    var fileSelect = document.getElementById("selected_file");
    if (fileSelect.files && fileSelect.files.length == 1) 
    {
        var file = fileSelect.files[0];
        formData.set("file", file, file.name);
        document.getElementById("ota_update_status").innerHTML = "Uploading " + file.name + ", Firmware Update in Progress...";
        var request = new XMLHttpRequest();
        request.upload.addEventListener("progress", updateProgress);
        request.open('POST', "/OTAupdate");
        request.responseType = "blob";
        request.send(formData);
    } else {
        window.alert('Select A File First')
    }
}

function updateProgress(oEvent) 
{
    if (oEvent.lengthComputable) { getUpdateStatus(); }
    else { window.alert('total size is unknown') }
}

function getUpdateStatus() 
{
    var xhr = new XMLHttpRequest();
    xhr.open('POST', "/OTAstatus", false);
    xhr.send('ota_update_status');
    if (xhr.readyState == 4 && xhr.status == 200) 
    {   
        var response = JSON.parse(xhr.responseText);
        document.getElementById("latest_firmware").innerHTML = response.compile_date + " - " + response.compile_time
        if (response.ota_update_status == 1) 
        {
            seconds = 10;
            otaRebootTimer();
        } 
        else if (response.ota_update_status == -1)
        {
            document.getElementById("ota_update_status").innerHTML = "!!! Upload Error !!!";
        }
    }
}

function otaRebootTimer() 
{   
    document.getElementById("ota_update_status").innerHTML = "OTA Firmware Update Complete. Rebooting in: " + seconds;
    if (--seconds == 0) 
    {
        clearTimeout(otaTimerVar);
        window.location.reload();
    } 
    else 
    {
        otaTimerVar = setTimeout(otaRebootTimer, 1000);
    }
}

/** Sensors **/
function getDHTSensorValues(){} // deshabilitado; usar temp de fan/state

function startDHTSensorInterval(){} // deshabilitado

/** WiFi **/
function stopWifiConnectStatusInterval()
{
    if (wifiConnectInterval != null)
    {
        clearInterval(wifiConnectInterval);
        wifiConnectInterval = null;
    }
}

function getWifiConnectStatus()
{
    var xhr = new XMLHttpRequest();
    xhr.open('POST', "/wifiConnectStatus", false);
    xhr.send('wifi_connect_status');
    if (xhr.readyState == 4 && xhr.status == 200)
    {
        var response = JSON.parse(xhr.responseText);
        document.getElementById("wifi_connect_status").innerHTML = "Connecting...";
        if (response.wifi_connect_status == 2)
        {
            document.getElementById("wifi_connect_status").innerHTML = "<h4 class='rd'>Failed to Connect.</h4>";
            stopWifiConnectStatusInterval();
        }
        else if (response.wifi_connect_status == 3)
        {
            document.getElementById("wifi_connect_status").innerHTML = "<h4 class='gr'>Connection Success!</h4>";
            stopWifiConnectStatusInterval();
        }
    }
}

function startWifiConnectStatusInterval()
{
    wifiConnectInterval = setInterval(getWifiConnectStatus, 2800);
}

function connectWifi()
{
    selectedSSID = $("#connect_ssid").val();
    pwd = $("#connect_pass").val();
    var requestData = {
      'selectedSSID': selectedSSID,
      'pwd': pwd,
      'timestamp': Date.now()
    };
    $.ajax({
      url: '/wifiConnect.json',
      dataType: 'json',
      method: 'POST',
      cache: false,
      data: JSON.stringify(requestData),
      contentType: 'application/json',
      success: function(response) { console.log(response); },
      error: function(xhr, status, error) { console.error(xhr.responseText); }
    });
}

function checkCredentials()
{
    errorList = "";
    credsOk = true;
    selectedSSID = $("#connect_ssid").val();
    pwd = $("#connect_pass").val();
    if (selectedSSID == "") { errorList += "<h4 class='rd'>SSID cannot be empty!</h4>"; credsOk = false; }
    if (pwd == "") { errorList += "<h4 class='rd'>Password cannot be empty!</h4>"; credsOk = false; }
    if (credsOk == false)
    {
        $("#wifi_connect_credentials_errors").html(errorList);
    }
    else
    {
        $("#wifi_connect_credentials_errors").html("");
        connectWifi();    
    }
}

function showPassword()
{
    var x = document.getElementById("connect_pass");
    if (x.type === "password") { x.type = "text"; }
    else { x.type = "password"; }
}

/** Fan control **/
function startFanPolling(){
    fetchFanState();
    setInterval(fetchFanState, 4000);
}

function fetchFanState(){
    $.getJSON('/fan/state', function(data){
        fanStateCache = data;
        $("#temperature_reading").text(data.temp.toFixed(1));
        $("#pir_state").text(data.pir ? "Detectado" : "Libre");
        $("#pwm_value").text(data.pwm.toFixed(1) + "%");
        var modeName = ['Manual','Automatico','Programado'][data.mode] || '--';
        $("#mode_value").text(modeName);
        pushTempSample(data.temp);

        var allowFanOverwrite = true;
        if (fanFormDirty) {
            if (Date.now() <= fanDirtyUntil) {
                allowFanOverwrite = false;
            } else {
                fanFormDirty = false;
            }
        }

        if (allowFanOverwrite) {
            $("input[name='fan_mode'][value="+data.mode+"]").prop('checked', true);
            updateModeUI(data.mode);
            $("#manual_pwm_slider").val(data.manual_pwm);
            $("#manual_pwm_label").text(data.manual_pwm.toFixed(0) + "%");
            $("#t_min").val(data.t_min);
            $("#t_max").val(data.t_max);
        } else {
            updateModeUI(parseInt($("input[name='fan_mode']:checked").val()));
        }

        // Sync program register checkboxes with active flag
        if (data.schedules) {
            for (var i = 0; i < Math.min(3, data.schedules.length); i++) {
                $("#reg_prog_"+i).prop('checked', !!data.schedules[i].active);
            }
        }

        renderSchedules(data.schedules);
        if (!scheduleFormDirty) {
            populateScheduleForm();
        }
    });
}

function applyFanConfig(){
    var mode = parseInt($("input[name='fan_mode']:checked").val());
    var pwm = parseFloat($("#manual_pwm_slider").val());
    var tmin = parseFloat($("#t_min").val());
    var tmax = parseFloat($("#t_max").val());
    $.ajax({url:'/fan/mode', method:'POST', data:JSON.stringify({'mode':mode}), contentType:'application/json'});
    $.ajax({url:'/fan/manual', method:'POST', data:JSON.stringify({'pwm':pwm}), contentType:'application/json'});
    $.ajax({url:'/fan/auto', method:'POST', data:JSON.stringify({'t_min':tmin,'t_max':tmax}), contentType:'application/json', success:function(){
        fanFormDirty = false;
    }});
}

function toggleProgramRegister(idx, active){
    if(!fanStateCache || !fanStateCache.schedules || !fanStateCache.schedules[idx]) return;
    var s = fanStateCache.schedules[idx];
    var payload = {
        index: idx,
        active: active,
        start_hour: s.start_hour,
        start_minute: s.start_minute,
        end_hour: s.end_hour,
        end_minute: s.end_minute,
        days_mask: s.days_mask,
        temp_0: s.temp_0,
        temp_100: s.temp_100
    };

    if (active && schedulesConflict(idx, payload)) {
        alert("Interferencia: este horario coincide con otro registro activo. Ajusta o desactiva uno.");
        $("#reg_prog_" + idx).prop('checked', false);
        return;
    }

    $.ajax({url:'/fan/schedule', method:'POST', data:JSON.stringify(payload), contentType:'application/json', success:function(){
        fanStateCache.schedules[idx].active = active;
    }});
}

function daysMaskFromUI(){
    var mask = 0;
    for(var i=0;i<7;i++){
        if($("#day_"+i).is(":checked")) mask |= (1<<i);
    }
    return mask;
}

function setDaysUI(mask){
    for(var i=0;i<7;i++){
        $("#day_"+i).prop('checked', (mask >> i) & 0x1);
    }
}

function populateScheduleForm(){
    if(!fanStateCache) return;
    var idx = parseInt($("#selectNumber").val()) - 1;
    var s = fanStateCache.schedules[idx];
    if(!s) return;
    $("#start_time").val(String(s.start_hour).padStart(2,'0')+":"+String(s.start_minute).padStart(2,'0'));
    $("#end_time").val(String(s.end_hour).padStart(2,'0')+":"+String(s.end_minute).padStart(2,'0'));
    $("#temp0").val(s.temp_0);
    $("#temp100").val(s.temp_100);
    $("#sched_active").prop('checked', s.active);
    setDaysUI(s.days_mask);
}

function renderSchedules(schedules){
    var list = $("#schedule_list");
    list.empty();
    for(var i=0;i<schedules.length;i++){
        var s = schedules[i];
        var days = maskToDaysText(s.days_mask);
        var label = `Reg ${i+1}: ${s.active?"On":"Off"} ${String(s.start_hour).padStart(2,'0')}:${String(s.start_minute).padStart(2,'0')} - ${String(s.end_hour).padStart(2,'0')}:${String(s.end_minute).padStart(2,'0')} | ${s.temp_0}-${s.temp_100}C | ${days}`;
        list.append(`<div class="reg-row"><span>${label}</span></div>`);
    }
}

function maskToDaysText(mask){
    const names = ['Dom','Lun','Mar','Mie','Jue','Vie','Sab'];
    var arr = [];
    for(var i=0;i<7;i++) if((mask>>i)&1) arr.push(names[i]);
    return arr.join(', ');
}

function send_schedule(){
    var idx = parseInt($("#selectNumber").val()) - 1;
    var start = $("#start_time").val().split(":" );
    var end = $("#end_time").val().split(":" );
    var payload = {
        index: idx,
        active: $("#sched_active").is(":checked"),
        start_hour: parseInt(start[0]),
        start_minute: parseInt(start[1]),
        end_hour: parseInt(end[0]),
        end_minute: parseInt(end[1]),
        days_mask: daysMaskFromUI(),
        temp_0: parseFloat($("#temp0").val()),
        temp_100: parseFloat($("#temp100").val())
    };

    if (schedulesConflict(idx, payload)) {
        alert("Interferencia: este horario coincide con otro registro activo. Ajusta o desactiva uno.");
        return;
    }


    $.ajax({url:'/fan/schedule', method:'POST', data:JSON.stringify(payload), contentType:'application/json', success:function(){
        scheduleFormDirty = false;
        fetchFanState();
    }});
}

/** Logs **/
function startLogsPolling(){
    fetchLogs();
    setInterval(fetchLogs, 6000);
}

function formatTs(ts){
    if(!ts) return '--';
    var d = new Date(ts*1000);
    return d.toLocaleString();
}

function fetchLogs(){
    $.getJSON('/fan/logs', function(data){
        var list = $("#log_list");
        list.empty();
        list.append('<div class="log-row log-head"><span>Inicio</span><span>Fin</span><span>Temp min/max</span><span>PWM max</span></div>');
        if(data.length === 0){
            list.append('<div class="status">Sin datos</div>');
            return;
        }
        data.forEach(function(e){
            list.append(`<div class="log-row"><span>${formatTs(e.start_ts)}</span><span>${formatTs(e.end_ts)}</span><span>${e.temp_min.toFixed(1)} / ${e.temp_max.toFixed(1)} C</span><span>${e.max_pwm.toFixed(1)}%</span></div>`);
        });
    });
}

/** Legacy hooks kept (toggle) **/
function toogle_led() 
{   
    $.ajax({ url: '/toogle_led.json', dataType: 'json', method: 'POST', cache: false });
}

