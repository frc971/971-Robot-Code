<!DOCTYPE HTML>
<html>
  <head>
    <title>971 Camera Code: Robot Stream</title>
    <style type="text/css">
      #body {
        display: block;
        margin: 0px;
        margin-top: 0px;
        margin-right: 0px;
        margin-bottom: 0px;
        margin-left: 0px;
      }
      #img {
        position: absolute;
        left: 50%;
        top: 0%;
        margin: 0 0 0 -320px;
      }
      #center {
        left: 50%;
        position: absolute;
        width: 2px;
        height: 100%;
        background-color: red;
      }
      #middle {
        top: 240px;
        margin-top: -1px;
        width: 100%;
        position: absolute;
        height: 2px;
        background-color: red;
      }
      #footer {
        top: 482px;
        left: 10px;
        position: absolute;
      }
      #center {
        margin-left: {{CENTER}}px;
      }
    </style>
  </head>
  <body id="body">
        <img id="img" src="http://{{HOST}}:9714" />
        <div id="center"></div>
        <div id="middle"></div>
        <div id="footer">
          <!--<form>
            <input type="button" value="Camera Controls"
           onclick="window.open('control.htm', 'Camera_Controls')">
   </form>-->
        </div>
  </body>
</html>
