package main

import (
	"bufio"
	"bytes"
	"encoding/json"
	"flag"
	"fmt"
	"github.com/buildkite/go-buildkite/buildkite"
	"io/ioutil"
	"log"
	"net/http"
	"os/exec"
	"regexp"
	"strings"
	"sync"
	"time"
)

type Commit struct {
	Sha1         string
	ChangeId     string
	ChangeNumber int
	Patchset     int
}

type State struct {
	// This mutex needs to be locked across anything which generates a uuid or accesses Commits.
	mu sync.Mutex
	// Mapping from build UUID from buildkite to the Commit information which triggered it.
	Commits map[string]Commit
	User    string
	Key     string
	// Webhook token expected to service requests
	Token string
}

type Approval struct {
	Type        string `json:"type"`
	Description string `json:"description"`
	Value       string `json:"value"`
	OldValue    string `json:"oldValue"`
}

type User struct {
	Name     string `json:"name"`
	Email    string `json:"email,omitempty"`
	Username string `json:"username,omitempty"`
}

type PatchSet struct {
	Number         int      `json:"number"`
	Revision       string   `json:"revision"`
	Parents        []string `json:"parents"`
	Ref            string   `json:"ref"`
	Uploader       User     `json:"uploader"`
	CreatedOn      int      `json:"createdOn"`
	Author         User     `json:"author"`
	Kind           string   `json:"kind,omitempty"`
	SizeInsertions int      `json:"sizeInsertions,omitempty"`
	SizeDeletions  int      `json:"sizeDeletions,omitempty"`
}

type Change struct {
	Project       string `json:"project"`
	Branch        string `json:"branch"`
	ID            string `json:"id"`
	Number        int    `json:"number"`
	Subject       string `json:"subject"`
	Owner         User   `json:"owner"`
	URL           string `json:"url"`
	CommitMessage string `json:"commitMessage"`
	CreatedOn     int    `json:"createdOn"`
	Status        string `json:"status"`
}

type ChangeKey struct {
	ID string `json:"id"`
}

type RefUpdate struct {
	OldRev  string `json:"oldRev"`
	NewRev  string `json:"newRev"`
	RefName string `json:"refName"`
	Project string `json:"project"`
}

type EventInfo struct {
	Author         *User      `json:"author"`
	Uploader       *User      `json:"uploader"`
	Reviewer       *User      `json:"reviewer"`
	Submitter      User       `json:"submitter,omitempty"`
	Approvals      []Approval `json:"approvals,omitempty"`
	Comment        string     `json:"comment,omitempty"`
	PatchSet       *PatchSet  `json:"patchSet"`
	Change         *Change    `json:"change"`
	Project        string     `json:"project"`
	RefName        string     `json:"refName"`
	ChangeKey      ChangeKey  `json:"changeKey"`
	RefUpdate      *RefUpdate `json:"refUpdate"`
	Type           string     `json:"type"`
	EventCreatedOn int        `json:"eventCreatedOn"`
}

// Simple application to poll Gerrit for events and trigger builds on buildkite when one happens.

// Handles a gerrit event and triggers buildkite accordingly.
func (s *State) handleEvent(eventInfo EventInfo, client *buildkite.Client) {
	// Only work on 971-Robot-Code
	if eventInfo.Project != "971-Robot-Code" {
		log.Printf("Ignoring project: '%s'\n", eventInfo.Project)
		return
	}

	// Find the change id, change number, patchset revision
	if eventInfo.Change == nil {
		log.Println("Failed to find Change")
		return
	}

	if eventInfo.PatchSet == nil {
		log.Println("Failed to find Change")
		return
	}

	log.Printf("Got a matching change of %s %s %d,%d\n",
		eventInfo.Change.ID, eventInfo.PatchSet.Revision, eventInfo.Change.Number, eventInfo.PatchSet.Number)

	for {

		// Triggering a build creates a UUID, and we can see events back from the webhook before the command returns.  Lock across the command so nothing access Commits while the new UUID is being added.
		s.mu.Lock()

		var user *User
		if eventInfo.Author != nil {
			user = eventInfo.Author
		} else if eventInfo.Uploader != nil {
			user = eventInfo.Uploader
		} else {
			log.Fatalf("Failed to find Author or Uploader")
		}

		// Trigger the build.
		if build, _, err := client.Builds.Create(
			"spartan-robotics", "971-robot-code", &buildkite.CreateBuild{
				Commit: eventInfo.PatchSet.Revision,
				Branch: eventInfo.Change.ID,
				Author: buildkite.Author{
					Name:  user.Name,
					Email: user.Email,
				},
				Env: map[string]string{
					"GERRIT_CHANGE_NUMBER": fmt.Sprintf("%d", eventInfo.Change.Number),
					"GERRIT_PATCH_NUMBER":  fmt.Sprintf("%d", eventInfo.PatchSet.Number),
				},
			}); err == nil {

			if build.ID != nil {
				log.Printf("Scheduled build %s\n", *build.ID)
				s.Commits[*build.ID] = Commit{
					Sha1:         eventInfo.PatchSet.Revision,
					ChangeId:     eventInfo.Change.ID,
					ChangeNumber: eventInfo.Change.Number,
					Patchset:     eventInfo.PatchSet.Number,
				}
			}
			s.mu.Unlock()

			if data, err := json.MarshalIndent(build, "", "\t"); err != nil {
				log.Fatalf("json encode failed: %s", err)
			} else {
				log.Printf("%s\n", string(data))
			}

			// Now remove the verified from Gerrit and post the link.
			cmd := exec.Command("ssh",
				"-p",
				"29418",
				"-i",
				s.Key,
				s.User+"@software.frc971.org",
				"gerrit",
				"review",
				"-m",
				fmt.Sprintf("\"Build Started: %s\"", *build.WebURL),
				"--verified",
				"0",
				fmt.Sprintf("%d,%d", eventInfo.Change.Number, eventInfo.PatchSet.Number))

			log.Printf("Running 'ssh -p 29418 -i %s %s@software.frc971.org gerrit review -m '\"Build Started: %s\"' --verified 0 %d,%d' and waiting for it to finish...",
				s.Key, s.User,
				*build.WebURL, eventInfo.Change.Number, eventInfo.PatchSet.Number)
			if err := cmd.Run(); err != nil {
				log.Printf("Command failed with error: %v", err)
			}
			return
		} else {
			s.mu.Unlock()
			log.Printf("Failed to trigger build: %s", err)
			log.Printf("Trying again in 30 seconds")
			time.Sleep(30 * time.Second)
		}
	}

}

type BuildkiteChange struct {
	ID     string `json:"id,omitempty"`
	Number int    `json:"number,omitempty"`
	URL    string `json:"url,omitempty"`
}

type Build struct {
	ID           string           `json:"id,omitempty"`
	GraphqlId    string           `json:"graphql_id,omitempty"`
	URL          string           `json:"url,omitempty"`
	WebURL       string           `json:"web_url,omitempty"`
	Number       int              `json:"number,omitempty"`
	State        string           `json:"state,omitempty"`
	Blocked      bool             `json:"blocked,omitempty"`
	BlockedState string           `json:"blocked_state,omitempty"`
	Message      string           `json:"message,omitempty"`
	Commit       string           `json:"commit"`
	Branch       string           `json:"branch"`
	Source       string           `json:"source,omitempty"`
	CreatedAt    string           `json:"created_at,omitempty"`
	ScheduledAt  string           `json:"scheduled_at,omitempty"`
	StartedAt    string           `json:"started_at,omitempty"`
	FinishedAt   string           `json:"finished_at,omitempty"`
	RebuiltFrom  *BuildkiteChange `json:"rebuilt_from,omitempty"`
}

type BuildkiteWebhook struct {
	Event string `json:"event"`
	Build Build  `json:"build"`
}

func (s *State) handle(w http.ResponseWriter, r *http.Request) {
	if r.URL.Path != "/" {
		http.Error(w, "404 not found.", http.StatusNotFound)
		return
	}

	switch r.Method {
	case "POST":
		if r.Header.Get("X-Buildkite-Token") != s.Token {
			http.Error(w, "Invalid token", http.StatusBadRequest)
			return
		}

		var data []byte
		var err error
		if data, err = ioutil.ReadAll(r.Body); err != nil {
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}

		log.Println(string(data))

		var webhook BuildkiteWebhook

		if err := json.Unmarshal(data, &webhook); err != nil {
			log.Fatalf("json decode failed: %s", err)
			http.Error(w, err.Error(), http.StatusBadRequest)
			return
		}

		// We've successfully received the webhook.  Spawn a goroutine in case the mutex is blocked so we don't block this thread.
		f := func() {
			if webhook.Event == "build.running" {
				if webhook.Build.RebuiltFrom != nil {
					s.mu.Lock()
					if c, ok := s.Commits[webhook.Build.RebuiltFrom.ID]; ok {
						log.Printf("Detected a rebuild of %s for build %s", webhook.Build.RebuiltFrom.ID, webhook.Build.ID)
						s.Commits[webhook.Build.ID] = c

						// And now remove the vote since the rebuild started.
						cmd := exec.Command("ssh",
							"-p",
							"29418",
							"-i",
							s.Key,
							s.User+"@software.frc971.org",
							"gerrit",
							"review",
							"-m",
							fmt.Sprintf("\"Build Started: %s\"", webhook.Build.WebURL),
							"--verified",
							"0",
							fmt.Sprintf("%d,%d", c.ChangeNumber, c.Patchset))

						log.Printf("Running 'ssh -p 29418 -i %s %s@software.frc971.org gerrit review -m '\"Build Started: %s\"' --verified 0 %d,%d' and waiting for it to finish...",
							s.Key, s.User,
							webhook.Build.WebURL, c.ChangeNumber, c.Patchset)
						if err := cmd.Run(); err != nil {
							log.Printf("Command failed with error: %v", err)
						}
					}
					s.mu.Unlock()
				}
			} else if webhook.Event == "build.finished" {
				var commit *Commit
				{
					s.mu.Lock()
					if c, ok := s.Commits[webhook.Build.ID]; ok {
						commit = &c
						// While we *should* delete this now from the map, that will prevent rebuilds from being mapped correctly.
						// Instead, leave it in the map indefinately.  For the number of builds we do, it should take quite a while to use enough ram to matter.  If that becomes an issue, we can either clean the list when a commit is submitted, or keep a fixed number of builds in the list and expire the oldest ones when it is time.
					}
					s.mu.Unlock()
				}

				if commit == nil {
					log.Printf("Unknown commit, ID: %s", webhook.Build.ID)
				} else {
					var verify string
					var status string

					if webhook.Build.State == "passed" {
						verify = "+1"
						status = "Succeeded"
					} else {
						verify = "-1"
						status = "Failed"
					}

					cmd := exec.Command("ssh",
						"-p",
						"29418",
						"-i",
						s.Key,
						s.User+"@software.frc971.org",
						"gerrit",
						"review",
						"-m",
						fmt.Sprintf("\"Build %s: %s\"", status, webhook.Build.WebURL),
						"--verified",
						verify,
						fmt.Sprintf("%d,%d", commit.ChangeNumber, commit.Patchset))

					log.Printf("Running 'ssh -p 29418 -i %s %s@software.frc971.org gerrit review -m '\"Build %s: %s\"' --verified %s %d,%d' and waiting for it to finish...",
						s.Key, s.User,
						status, webhook.Build.WebURL, verify, commit.ChangeNumber, commit.Patchset)
					if err := cmd.Run(); err != nil {
						log.Printf("Command failed with error: %v", err)
					}

				}
				if webhook.Build.State == "passed" {
					log.Printf("Passed build %s: %s", webhook.Build.ID, webhook.Build.Commit)
				} else {
					log.Printf("Failed build %s: %s", webhook.Build.ID, webhook.Build.Commit)
				}
			}
		}

		go f()

		log.Printf("%s: %s %s %s\n", webhook.Event, webhook.Build.ID, webhook.Build.Commit, webhook.Build.Branch)

		fmt.Fprintf(w, "")

	default:
		internalError := http.StatusInternalServerError
		http.Error(w, "Invalid method", internalError)
		log.Printf("Invalid method %s", r.Method)
	}
}

func main() {
	apiToken := flag.String("token", "", "API token")
	webhookToken := flag.String("webhook_token", "", "Expected webhook token")
	user := flag.String("user", "buildkite", "User to be in gerrit")
	key := flag.String("key", "~/.ssh/buildkite", "SSH key to use to connect to gerrit")
	debug := flag.Bool("debug", false, "Enable debugging")

	flag.Parse()

	state := State{
		Key:     *key,
		User:    *user,
		Commits: make(map[string]Commit),
		Token:   *webhookToken,
	}

	f := func() {
		http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
			state.handle(w, r)
		})
		log.Println("Starting webhook server on 10005\n")
		if err := http.ListenAndServe(":10005", nil); err != nil {
			log.Fatal(err)
		}
	}

	if *apiToken == "nope" {
		log.Println("Only starting server")
		f()
	} else {
		go f()
	}

	config, err := buildkite.NewTokenConfig(*apiToken, *debug)

	if err != nil {
		log.Fatalf("client config failed: %s", err)
	}

	client := buildkite.NewClient(config.Client())

	for {
		args := fmt.Sprintf("-o ServerAliveInterval=10 -o ServerAliveCountMax=3 -i %s -p 29418 %s@software.frc971.org gerrit stream-events", state.Key, state.User)
		cmd := exec.Command("ssh", strings.Split(args, " ")...)

		stdout, _ := cmd.StdoutPipe()
		cmd.Start()

		scanner := bufio.NewScanner(stdout)
		scanner.Split(bufio.ScanLines)
		for scanner.Scan() {
			m := scanner.Text()

			log.Println(m)

			var eventInfo EventInfo
			dec := json.NewDecoder(bytes.NewReader([]byte(m)))
			dec.DisallowUnknownFields()

			if err := dec.Decode(&eventInfo); err != nil {
				log.Printf("Failed to parse JSON: %e\n", err)
				continue
			}

			log.Printf("Got an event of type: '%s'\n", eventInfo.Type)
			switch eventInfo.Type {
			case "assignee-changed":
			case "change-abandoned":
			case "change-deleted":
			case "change-merged":
				// TODO(austin): Trigger a master build?
			case "change-restored":
			case "comment-added":
				if matched, _ := regexp.MatchString(`(?m)^retest$`, eventInfo.Comment); !matched {
					continue
				}

				state.handleEvent(eventInfo, client)
			case "dropped-output":
			case "hashtags-changed":
			case "project-created":
			case "patchset-created":
				state.handleEvent(eventInfo, client)
			case "ref-updated":
			case "reviewer-added":
			case "reviewer-deleted":
			case "topic-changed":
			case "wip-state-changed":
			case "private-state-changed":
			case "vote-deleted":
			default:
				log.Println("Unknown case")
			}
		}
		cmd.Wait()
	}
}
