const express = require("express");
const axios = require("axios");
const bodyParser = require("body-parser");
const dotenv = require("dotenv");
const cors = require("cors");

dotenv.config();

const app = express();
const port = process.env.PORT || 3000;
app.use(cors());

const getToken = async () => {
  const url = "https://notehub.io/oauth2/token";
  const clientId = process.env.CLIENT_ID;
  const clientSecret = process.env.CLIENT_SECRET;

  const data = new URLSearchParams();
  data.append("grant_type", "client_credentials");
  data.append("client_id", clientId);
  data.append("client_secret", clientSecret);

  try {
    const response = await axios.post(url, data, {
      headers: {
        "Content-Type": "application/x-www-form-urlencoded",
      },
    });

    const accessToken = response.data.access_token;
    console.log("Token:", accessToken);

    return accessToken;

    // Now you can use the accessToken variable as needed.
  } catch (error) {
    console.error("Error:", error);
  }
};

app.use(bodyParser.json());

// Endpoint to start the OAuth2 process
app.post("/execute", async (req, res) => {
  try {
    const accessToken = await getToken();
    console.log("Access token", accessToken);

    // Get the coordinates from the request body
    const coordinates = req.body.coordinates;

    // Make the API request using the acquired access token and coordinates
    const apiEndpoint = `https://api.notefile.net/v1/projects/${process.env.PROJECT_UID}/devices/${process.env.DEVICE_UID}/notes/data.qi`;
    const apiRequestBody = { body: { coordinates: coordinates } };
    const apiResponse = await axios.post(apiEndpoint, apiRequestBody, {
      headers: {
        Authorization: `Bearer ${accessToken}`,
      },
    });

    res.status(apiResponse.status).json(apiResponse.data);
  } catch (error) {
    console.error(error);
    res.status(500).json({ error: "Internal Server Error" });
  }
});

app.listen(port, () => {
  console.log(`Server is running on http://localhost:${port}`);
});
