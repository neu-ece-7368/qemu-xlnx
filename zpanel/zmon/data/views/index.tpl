<!DOCTYPE html>
<html>
  <head>
    <title>ZPanel</title>
    <link href="/css/style.css" rel='stylesheet' type='text/css' />
    <script src="/js/jquery.min.js"></script>
    <script>
      var refreshTimer;
      function startRefreshing()
      {
      refreshTimer = setInterval(refreshStatus, 100);
      }

      function stopRefreshing()
      {
      clearInterval(refreshTimer);
      }

      function refreshStatus()
      {
      $.ajax({
      method: "GET",
      url: "/state/leds",
      success: function(result) {
      if (result.status == "ok") {
      $.each(result.data,
      function(key, val) {
      setLEDState(key, val);
      });
      }
      }
      });
      }

      function setLEDState(led, state)
      {
      if (state >= 75) {
      $("#ledstate-"+led).addClass("red");
      }
      else if (state < 75 && state > 25) {
      $("#ledstate-"+led).addClass("lightRed");
      }
      else if (state <= 25 && state > 0){
      $("#ledstate-"+led).addClass("orange");
      }
      else
      {
      $("#ledstate-"+led).removeClass("red");
      $("#ledstate-"+led).removeClass("lightRed");
      $("#ledstate-"+led).removeClass("orange");
      }
      }

      function swToggle(sw)
      {
      if ($("#dip-"+sw).prop("checked"))
      {
      $.ajax({
      method: "POST",
      url: "/state/sw",
      data: { swn: sw, value: 1 },
      dataType: "json"
      });
      }
      else
      {
      $.ajax({
      method: "POST",
      url: "/state/sw",
      data: { swn: sw, value: 0 },
      dataType: "json"
      });
      }
      }
    </script>
  </head>

  <body onload="startRefreshing()">
  <div id="content">
  <h1>LEDs and Switches</h1>
  <table class="ledsw">
    <tr>
      <td><div id="ledstate-7" class="circle">LD7</div></td>
      <td><div id="ledstate-6" class="circle">LD6</div></td>
      <td><div id="ledstate-5" class="circle">LD5</div></td>
      <td><div id="ledstate-4" class="circle">LD4</div></td>
      <td><div id="ledstate-3" class="circle">LD3</div></td>
      <td><div id="ledstate-2" class="circle">LD2</div></td>
      <td><div id="ledstate-1" class="circle">LD1</div></td>
      <td><div id="ledstate-0" class="circle">LD0</div></td>
    </tr>
    <tr>
      <td>
        <label class="switch">
          <input id="dip-7" type="checkbox" onclick="swToggle(7)">
          <span class="slider"></span>
        </label>
      </td>
            <td>
        <label class="switch">
          <input id="dip-6" type="checkbox" onclick="swToggle(6)">
          <span class="slider"></span>
        </label>
      </td>
            <td>
        <label class="switch">
          <input id="dip-5" type="checkbox" onclick="swToggle(5)">
          <span class="slider"></span>
        </label>
      </td>
            <td>
        <label class="switch">
          <input id="dip-4" type="checkbox" onclick="swToggle(4)">
          <span class="slider"></span>
        </label>
      </td>
            <td>
        <label class="switch">
          <input id="dip-3" type="checkbox" onclick="swToggle(3)">
          <span class="slider"></span>
        </label>
      </td>
            <td>
        <label class="switch">
          <input id="dip-2" type="checkbox" onclick="swToggle(2)">
          <span class="slider"></span>
        </label>
      </td>
            <td>
        <label class="switch">
          <input id="dip-1" type="checkbox" onclick="swToggle(1)">
          <span class="slider"></span>
        </label>
      </td>
      <td>
        <label class="switch">
          <input id="dip-0" type="checkbox" onclick="swToggle(0)">
          <span class="slider"></span>
        </label>
      </td>
    </tr>
  </table>
</div>
  </body>
</html>
