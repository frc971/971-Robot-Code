package main

import (
	"bytes"
	"fmt"
	"image"
	"image/jpeg"
	_ "image/png"
	"log"
	"os"
	"path/filepath"
	"sort"
)

// Chrome plays back MJPEG files at a (hard-coded) 30 fps.
const CHROME_FAKE_VIDEO_FPS = 30

// For how many seconds to display a single image.
const IMAGE_DURATION = 3

// For how many frames (at CHROME_FAKE_VIDEO_FPS) to display a single image.
const IMAGE_DURATION_FRAMES = int(CHROME_FAKE_VIDEO_FPS * IMAGE_DURATION)

func checkErr(err error, message string) {
	if err != nil {
		log.Println(message)
		log.Fatal(err)
	}
}

func main() {
	output_dir := os.Getenv("TEST_UNDECLARED_OUTPUTS_DIR")

	// The output file is at a fixed path as expected by
	// `tools/build_rules/js/cypress.config.js`.
	outName := output_dir + "/fake_camera.mjpeg"

	// The Cypress test is expected to dump all the screenshots in this
	// directory.
	screenshotDir := output_dir + "/screenshots/scouting_qrcode_test.cy.js"
	log.Printf("Looking for screenshots in %s", screenshotDir)

	// Create a movie from images.
	matches, err := filepath.Glob(screenshotDir + "/qrcode_*_screenshot.png")
	checkErr(err, "Failed to glob for the screenshots")
	sort.Strings(matches)

	log.Println("Found images:", matches)
	if len(matches) < 2 {
		// For the purposes of the test, we expect at least 2 QR codes.
		// If something goes wrong, then this is an opportunity to bail
		// early.
		log.Fatalf("Only found %d images", len(matches))
	}

	mjpeg, err := os.Create(outName)
	checkErr(err, "Failed to open output file")
	defer mjpeg.Close()

	// MJPEG is litterally a bunch of JPEGs concatenated together. Read in
	// each frame and append it to the output file.
	for _, name := range matches {
		reader, err := os.Open(name)
		checkErr(err, "Could not open "+name)
		defer reader.Close()

		img, _, err := image.Decode(reader)
		checkErr(err, "Could not decode image")

		buffer := &bytes.Buffer{}
		checkErr(jpeg.Encode(buffer, img, nil), "Failed to encode as jpeg")

		// In order to show a single picture for 1 second, we need to
		// inject CHROME_FAKE_VIDEO_FPS copies of the same image.
		for i := 0; i < IMAGE_DURATION_FRAMES; i++ {
			_, err = mjpeg.Write(buffer.Bytes())
			checkErr(err, "Failed to write to mjpeg")
		}
	}

	fmt.Printf("%s was written successfully.\n", outName)
}
