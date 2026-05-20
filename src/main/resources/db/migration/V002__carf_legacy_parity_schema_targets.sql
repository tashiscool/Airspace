CREATE TABLE IF NOT EXISTS ops_altrv_message (
  id UUID PRIMARY KEY,
  mission_id UUID REFERENCES ops_mission(id) ON DELETE SET NULL,
  reservation_id UUID REFERENCES ops_reservation(id) ON DELETE SET NULL,
  activity_name VARCHAR(256),
  message_type VARCHAR(64),
  parser_status VARCHAR(64) NOT NULL DEFAULT 'UNPARSED',
  avana_at TIMESTAMP WITH TIME ZONE,
  tas VARCHAR(128),
  project_officer VARCHAR(256),
  alternate_project_officer VARCHAR(256),
  artccs_concerned TEXT,
  additional_info TEXT,
  raw_text TEXT,
  diagnostics_json TEXT,
  parsed_json TEXT,
  version BIGINT NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_altrv_callsign (
  id UUID PRIMARY KEY,
  altrv_message_id UUID NOT NULL REFERENCES ops_altrv_message(id) ON DELETE CASCADE,
  callsign VARCHAR(64) NOT NULL,
  aircraft_count INTEGER,
  sequence_index INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS ops_altrv_aircraft_type (
  id UUID PRIMARY KEY,
  altrv_message_id UUID NOT NULL REFERENCES ops_altrv_message(id) ON DELETE CASCADE,
  type_designator VARCHAR(64) NOT NULL,
  equipment VARCHAR(128),
  aircraft_count INTEGER,
  sequence_index INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS ops_altrv_departure (
  id UUID PRIMARY KEY,
  altrv_message_id UUID NOT NULL REFERENCES ops_altrv_message(id) ON DELETE CASCADE,
  callsign VARCHAR(64),
  location VARCHAR(128),
  etd_at TIMESTAMP WITH TIME ZONE,
  avana_at TIMESTAMP WITH TIME ZONE,
  adimis_text TEXT,
  sequence_index INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS ops_altrv_route_group (
  id UUID PRIMARY KEY,
  altrv_message_id UUID NOT NULL REFERENCES ops_altrv_message(id) ON DELETE CASCADE,
  group_name VARCHAR(128),
  route_family VARCHAR(64),
  source_text TEXT,
  sequence_index INTEGER NOT NULL DEFAULT 0,
  version BIGINT NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_altrv_route (
  id UUID PRIMARY KEY,
  route_group_id UUID NOT NULL REFERENCES ops_altrv_route_group(id) ON DELETE CASCADE,
  route_name VARCHAR(128),
  route_family VARCHAR(64) NOT NULL,
  lower_altitude_feet DOUBLE PRECISION,
  upper_altitude_feet DOUBLE PRECISION,
  effective_start TIMESTAMP WITH TIME ZONE,
  effective_end TIMESTAMP WITH TIME ZONE,
  source_text TEXT,
  sequence_index INTEGER NOT NULL DEFAULT 0,
  version BIGINT NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_altrv_fix_time (
  id UUID PRIMARY KEY,
  route_id UUID REFERENCES ops_altrv_route(id) ON DELETE CASCADE,
  event_id UUID,
  identifier VARCHAR(128),
  latitude DOUBLE PRECISION,
  longitude DOUBLE PRECISION,
  radial_degrees DOUBLE PRECISION,
  dme_nm DOUBLE PRECISION,
  elapsed_minutes INTEGER,
  absolute_time TIMESTAMP WITH TIME ZONE,
  is_last_fix BOOLEAN NOT NULL DEFAULT FALSE,
  source_text TEXT,
  sequence_index INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS ops_altrv_route_event (
  id UUID PRIMARY KEY,
  route_id UUID REFERENCES ops_altrv_route(id) ON DELETE CASCADE,
  event_type VARCHAR(64) NOT NULL,
  callsign VARCHAR(64),
  fix_time_id UUID REFERENCES ops_altrv_fix_time(id) ON DELETE SET NULL,
  area_id UUID,
  lower_altitude_feet DOUBLE PRECISION,
  upper_altitude_feet DOUBLE PRECISION,
  duration_seconds INTEGER,
  source_text TEXT,
  metadata_json TEXT,
  sequence_index INTEGER NOT NULL DEFAULT 0,
  version BIGINT NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL
);

ALTER TABLE ops_altrv_fix_time
  ADD CONSTRAINT fk_ops_altrv_fix_time_event
  FOREIGN KEY (event_id) REFERENCES ops_altrv_route_event(id) ON DELETE CASCADE;

CREATE TABLE IF NOT EXISTS ops_altrv_area (
  id UUID PRIMARY KEY,
  altrv_message_id UUID REFERENCES ops_altrv_message(id) ON DELETE CASCADE,
  route_event_id UUID REFERENCES ops_altrv_route_event(id) ON DELETE SET NULL,
  area_type VARCHAR(64) NOT NULL,
  width_nm DOUBLE PRECISION,
  radius_nm DOUBLE PRECISION,
  center_identifier VARCHAR(128),
  source_text TEXT,
  geometry_json TEXT,
  sequence_index INTEGER NOT NULL DEFAULT 0,
  version BIGINT NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL
);

ALTER TABLE ops_altrv_route_event
  ADD CONSTRAINT fk_ops_altrv_route_event_area
  FOREIGN KEY (area_id) REFERENCES ops_altrv_area(id) ON DELETE SET NULL;

CREATE TABLE IF NOT EXISTS ops_altrv_area_point (
  id UUID PRIMARY KEY,
  area_id UUID NOT NULL REFERENCES ops_altrv_area(id) ON DELETE CASCADE,
  identifier VARCHAR(128),
  latitude DOUBLE PRECISION,
  longitude DOUBLE PRECISION,
  radial_degrees DOUBLE PRECISION,
  dme_nm DOUBLE PRECISION,
  sequence_index INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS ops_altrv_exit (
  id UUID PRIMARY KEY,
  route_id UUID NOT NULL REFERENCES ops_altrv_route(id) ON DELETE CASCADE,
  exit_type VARCHAR(64) NOT NULL,
  fix_identifier VARCHAR(128),
  callsign VARCHAR(64),
  elapsed_minutes INTEGER,
  source_text TEXT,
  sequence_index INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS ops_altrv_destination (
  id UUID PRIMARY KEY,
  route_id UUID REFERENCES ops_altrv_route(id) ON DELETE CASCADE,
  exit_id UUID REFERENCES ops_altrv_exit(id) ON DELETE CASCADE,
  location VARCHAR(128),
  destination_type VARCHAR(64),
  elapsed_minutes INTEGER,
  source_text TEXT,
  sequence_index INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS ops_reservation_note (
  id UUID PRIMARY KEY,
  reservation_id UUID REFERENCES ops_reservation(id) ON DELETE CASCADE,
  username VARCHAR(128),
  note_text TEXT NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_reservation_image (
  id UUID PRIMARY KEY,
  reservation_id UUID REFERENCES ops_reservation(id) ON DELETE CASCADE,
  label VARCHAR(256),
  image_bytes BYTEA,
  image_text TEXT,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_airspace_reference (
  id UUID PRIMARY KEY,
  code VARCHAR(64) NOT NULL UNIQUE,
  name VARCHAR(256),
  struct_type VARCHAR(64),
  metadata_json TEXT
);

CREATE TABLE IF NOT EXISTS ops_message_recipient (
  id UUID PRIMARY KEY,
  message_id UUID NOT NULL REFERENCES ops_message(id) ON DELETE CASCADE,
  recipient_type VARCHAR(32) NOT NULL,
  address VARCHAR(256) NOT NULL,
  display_name VARCHAR(256)
);

CREATE TABLE IF NOT EXISTS ops_message_airspace_info (
  id UUID PRIMARY KEY,
  reservation_id UUID REFERENCES ops_reservation(id) ON DELETE CASCADE,
  message_id UUID REFERENCES ops_message(id) ON DELETE SET NULL,
  airspace_code VARCHAR(64),
  apreq_date_sent VARCHAR(128),
  approval_date_sent VARCHAR(128),
  apreq_body TEXT,
  approval_body TEXT,
  metadata_json TEXT
);

CREATE TABLE IF NOT EXISTS ops_group_separation (
  id UUID PRIMARY KEY,
  group_name VARCHAR(128) NOT NULL,
  lateral_nm DOUBLE PRECISION,
  vertical_feet DOUBLE PRECISION,
  longitudinal_minutes DOUBLE PRECISION,
  metadata_json TEXT
);

CREATE TABLE IF NOT EXISTS ops_aircraft_validator (
  id UUID PRIMARY KEY,
  manufacturer VARCHAR(128),
  model VARCHAR(128),
  type_designator VARCHAR(64) NOT NULL UNIQUE,
  engine_count VARCHAR(32),
  engine_type VARCHAR(64),
  wake_turbulence_category VARCHAR(32),
  description TEXT
);

CREATE TABLE IF NOT EXISTS ops_preferred_navaid (
  id UUID PRIMARY KEY,
  identifier VARCHAR(64) NOT NULL,
  reference_point_id UUID REFERENCES ops_reference_point(id) ON DELETE CASCADE,
  priority INTEGER NOT NULL DEFAULT 0,
  country VARCHAR(64),
  metadata_json TEXT,
  UNIQUE(identifier, reference_point_id)
);

CREATE INDEX IF NOT EXISTS idx_ops_altrv_message_mission ON ops_altrv_message(mission_id, created_at);
CREATE INDEX IF NOT EXISTS idx_ops_altrv_route_group_message ON ops_altrv_route_group(altrv_message_id, sequence_index);
CREATE INDEX IF NOT EXISTS idx_ops_altrv_route_group ON ops_altrv_route(route_group_id, sequence_index);
CREATE INDEX IF NOT EXISTS idx_ops_altrv_fix_time_route ON ops_altrv_fix_time(route_id, sequence_index);
CREATE INDEX IF NOT EXISTS idx_ops_altrv_event_route ON ops_altrv_route_event(route_id, event_type, sequence_index);
CREATE INDEX IF NOT EXISTS idx_ops_altrv_area_message ON ops_altrv_area(altrv_message_id, area_type);
CREATE INDEX IF NOT EXISTS idx_ops_message_recipient_message ON ops_message_recipient(message_id, recipient_type);
