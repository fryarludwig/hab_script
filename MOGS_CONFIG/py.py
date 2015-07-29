import sys

_googleMapsHtml = """
<!DOCTYPE html>
<html>
	<head>
	<style>
		html, body, #map-canvas {
		height: 100%;
		margin: 0px;
		padding: 0px
		}
	</style>
	<script src="https://maps.googleapis.com/maps/api/js?v=3.exp&sensor=false&libraries=drawing"></script>
	<script>
		var map;
		var VEHICLES_TO_PLOT = 4
		var myCenter = new google.maps.LatLng(36.8623, -121.0413);
		
		var vehicleWaypointArray = [];
		var vehicleMarkerArray = [];
		var spotMarkerArray = [];
		var manualMarkerArray = [];
		
		var vehiclePathColors = ["#FF0000",
								"#007FFF",
								"#006600",
								"#FF9933"];
		
		var vehicleIconNames = ["http://fryarludwig.com/wp-content/uploads/2015/07/hotairbaloon.png",
								"http://maps.google.com/mapfiles/kml/paddle/1-lv.png",
								"http://maps.google.com/mapfiles/kml/paddle/2-lv.png",
								"http://maps.google.com/mapfiles/kml/paddle/3-lv.png"];
		
		var vehicleSpotIcons = ["http://fryarludwig.com/wp-content/uploads/2015/07/letter_b.png",
								"http://fryarludwig.com/wp-content/uploads/2015/07/number_1.png",
								"http://fryarludwig.com/wp-content/uploads/2015/07/number_2.png",
								"http://fryarludwig.com/wp-content/uploads/2015/07/number_3.png"];
		
		for (i = 0; i < VEHICLES_TO_PLOT; i++)
		{
			vehicleWaypointArray.push([]);
			vehicleMarkerArray.push(new google.maps.Marker());
		}
		
		function initialize() 
		{
			var mapOptions = 
			{
				center: myCenter,
				zoom: 9,
				mapTypeId: google.maps.MapTypeId.ROADMAP
			};
	
			map = new google.maps.Map(document.getElementById('map-canvas'), mapOptions);
		}
				
		function addVehicleWaypoint(index, lat, lng, time)
		{
			var vehiclePosition = new google.maps.LatLng(lat, lng);
			
			if (vehicleWaypointArray[index].length > 1)
			{
				vehicleWaypointArray[index].shift();
			}
			
			vehicleWaypointArray[index].push(vehiclePosition);
			
			vehicleMarkerArray[index].setMap(null);
			
			vehicleMarkerArray[index] = new google.maps.Marker(
												{position:vehiclePosition,
												icon: vehicleIconNames[index],
												title: time,
												map:map});
												
			vehicleMarkerArray[index].setMap(map);
			
			var line = new google.maps.Polyline
			({
				path:vehicleWaypointArray[index],
				strokeColor:vehiclePathColors[index],
				strokeOpacity:0.8,
				strokeWeight:2,
				map:map
			});
		}		
					
		function addSpotMarker(index, lat, lng, timestamp)
		{
			var spotMarkerPosition = new google.maps.LatLng(lat, lng);
			
			spotMarker = new google.maps.Marker({position:spotMarkerPosition,
												icon:vehicleSpotIcons[index],
												title:timestamp,
												map:map});
			
			spotMarker.setMap(map);
		}
		
		function addMarkerManually(lat, lng, note)
		{
			var pos = new google.maps.LatLng(lat, lng);
			
			var currMarker = new google.maps.Marker({position:pos,
												title:note,
												map:map});
			
			currMarker.setMap(map);
			
			manualMarkerArray.push(currMarker);
		}
		
		function clearManualMarkers()
		{
			for (i = 0; i < manualMarkerArray.length; i++)
			{
				manualMarkerArray[i].setMap(null);
			}
			
			manualMarkerArray = [];
		}
		
		google.maps.event.addDomListener(window, 'load', initialize);

	</script>
	</head>
	<body>
	<div id="map-canvas"></div>
	</body>
</html>
"""
