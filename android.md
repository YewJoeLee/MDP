

<table>
 <tr>
  <th>No.</th>
  <th>C. Android Remote Controller Module<br/>Functional Specifications</th>
  <th>MDP<br/>Supervisor<br/>Signature /<br/>Date</th>
  <th>Name of<br/>Student<br/>Contributor</th>
 </tr>
 <tr>
  <td>C.1</td>
  <td>The Android application (AA) is able to transmit and<br/>receive text strings over the Bluetooth serial<br/>communication link.<br/>Note: You can use the AMD tool to help verify that your AA has<br/>successfully achieved bi-directional data transfer.</td>
  <td></td>
  <td></td>
 </tr>
 <tr>
  <td>C.2</td>
  <td>Functional graphical user interface (GUI) that is able<br/>to initiate the scanning, selection and connection with a<br/>Bluetooth device.<br/>E.g. when the Connect button is touched, a list of available devices is<br/>presented to the user for selection. Once a device is selected, a<br/>connection is established with the device. You can use C.1 to show<br/>evidence of a successful connection.</td>
  <td></td>
  <td></td>
 </tr>
 <tr>
  <td>C.3</td>
  <td>Functional GUI that provides interactive control of the<br/>robot movement via the Bluetooth link (e.g. move<br/>forward, left and right). The interactive control of the robot<br/>movement can be done using several labeled buttons (minimal<br/>requirement), appropriate touch gestures, button cum device tilt or any<br/>other method you can think of. You can use the AMD tool to<br/>demonstrate control of the virtual robot movement.<br/>Caution: Manually entering different string commands in a text box to<br/>control the robot movement is not a valid implementation of this<br/>requirement.</td>
  <td></td>
  <td></td>
 </tr>
 <tr>
  <td>C.4</td>
  <td>Functional GUI that shows remote update &amp; status<br/>messages (e.g. ready to start, looking for target 2, etc). You<br/>can implement this using a TextView box (minimal requirement). You<br/>can use the AMD tool to simulate information update by devising your<br/>own string-based protocol representing the various possible status of<br/>your robot.<br/>Note: Your TextView box must only display selective information and<br/>not all the text data that is being streamed to Android tablet.</td>
  <td></td>
  <td></td>
 </tr>
 <tr>
  <td>C.5</td>
  <td>2D display of the exploration arena with obstacles and<br/>the robot's location.<br/>E.g. you can create a drawing canvas on your GUI where square<br/>numbered obstacle blocks (from 1, 2, 3,..n) can be drawn within a<br/>bounded exploration arena The number text drawn inside your<br/>obstacle should be in small white colored fonts and it represents your<br/>assigned number to each new obstacle added to your map (e.g or<br/>). A robot is also drawn at a specified coordinate (x,y) and its<br/>facing direction (N,S,E,W) can be clearly inferred from the robot icon<br/>displayed.</td>
  <td></td>
  <td></td>
 </tr>
 <tr>
  <td>C.6</td>
  <td>Interactive movement and placement of obstacles in<br/>map.<br/>You GUI must allow you to interactively place the square obstacles<br/>into the touch interactions on area. You must</td>
  <td></td>
  <td></td>
 </tr>
</table>




# CE/CZ3004 - Multi-disciplinary Design Project (MDP) (Assessment Component)


<table>
 <tr>
  <th></th>
  <td>also allow these obstacles in the map to be moved around within the<br/>map through a "touch and drag" interaction. Dragging the obstacle<br/>outside the map area will remove the obstacle from your map. Once<br/>the positioning of the obstacle is completed and the finger is lifted, the<br/>(x,y) coordinates and number assigned to the obstacle is transmitted<br/>out via the Bluetooth channel. You are free to devise the string format<br/>for this information.</td>
 </tr>
 <tr>
  <th>C.7</th>
  <td>Interactive annotation of the face of the obstacle where<br/>the target image is located.<br/>Your GUI must provide functionality that will allow you to indicate<br/>which of the side of any particular obstacle touched has the target<br/>image. If your obstacles are too small to touch one of the four sides,<br/>device another method that will allow you to do this task. Any<br/>alternative method to specify which side of the four faces has the<br/>target image must still be a touch-based interaction. Once the target<br/>face has been registered, the appearance of the obstacle must change<br/>to indicate which obstacle face has the target image (e.g. ) and<br/>the target face and obstacle coordinate must be communicated via the<br/>Bluetooth channel. You are free to devise the string format for this<br/>information.</td>
 </tr>
 <tr>
  <th>C.8</th>
  <td>Robust connectivity with Bluetooth device.<br/>Your Android application (AA) must not hang up if connectivity with<br/>the Bluetooth device is temporarily lost (e.g. by executing a<br/>Disconnect at the AMD tool after connection has been established).<br/>Your AA should automatically re-established connection<br/>automatically once the Bluetooth device connects with the AA again<br/>(e.g. by executing a Connect again at the AMD tool after connection<br/>was earlier broken with a Disconnect).</td>
 </tr>
 <tr>
  <th>C.9</th>
  <td>Displaying Image Target ID on Obstacle Blocks in the<br/>Map.<br/>The appearance of any numbered obstacle block can be changed to<br/>display a Target ID (in large white fonts) when the Bluetooth channel<br/>receives the string "TARGET, &lt;Obstacle Number&gt;, &lt;Target ID&gt;".<br/>For example, the obstacle block appearance changes to , ,<br/>etc. The face in which the target image is located is displayed with a<br/>thick visible line of a distinguishing color (e.g. from to<br/>when target ID of 4 is received )</td>
 </tr>
 <tr>
  <th>C.10</th>
  <td>Updating Position and Facing Direction of Robot in the<br/>Map.<br/>The position of the robot and the direction the robot is facing can be<br/>updated in the map of your Android tablet when the Bluetooth channel<br/>receives the string "ROBOT, &lt;x&gt;, &lt;y&gt;, &lt;direction&gt;", where &lt;x&gt; and<br/>&lt;y&gt; are valid integer coordinates in your map and &lt;direction&gt; is any<br/>one of four directions (N, S, E, W).</td>
 </tr>
</table>


## Page 2

