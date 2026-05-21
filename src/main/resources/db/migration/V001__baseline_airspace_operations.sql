CREATE TABLE IF NOT EXISTS ops_user (
  id UUID PRIMARY KEY,
  username VARCHAR(128) NOT NULL UNIQUE,
  display_name VARCHAR(256),
  password_hash VARCHAR(512) NOT NULL,
  active BOOLEAN NOT NULL DEFAULT TRUE,
  version BIGINT NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_role (
  id UUID PRIMARY KEY,
  name VARCHAR(128) NOT NULL UNIQUE,
  description TEXT
);

CREATE TABLE IF NOT EXISTS ops_user_role (
  user_id UUID NOT NULL REFERENCES ops_user(id) ON DELETE CASCADE,
  role_name VARCHAR(128) NOT NULL,
  PRIMARY KEY (user_id, role_name)
);

CREATE TABLE IF NOT EXISTS ops_mission (
  id UUID PRIMARY KEY,
  mission_number VARCHAR(128) NOT NULL UNIQUE,
  title VARCHAR(256),
  status VARCHAR(64) NOT NULL,
  raw_text TEXT,
  locked_by VARCHAR(128),
  locked_at TIMESTAMP WITH TIME ZONE,
  version BIGINT NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_reservation (
  id UUID PRIMARY KEY,
  mission_id UUID REFERENCES ops_mission(id) ON DELETE CASCADE,
  reservation_key VARCHAR(128),
  state VARCHAR(64) NOT NULL,
  raw_text TEXT,
  lower_altitude_feet DOUBLE PRECISION,
  upper_altitude_feet DOUBLE PRECISION,
  effective_start TIMESTAMP WITH TIME ZONE,
  effective_end TIMESTAMP WITH TIME ZONE,
  locked_by VARCHAR(128),
  locked_at TIMESTAMP WITH TIME ZONE,
  last_analysis_json TEXT,
  version BIGINT NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_message (
  id UUID PRIMARY KEY,
  mission_id UUID REFERENCES ops_mission(id) ON DELETE SET NULL,
  reservation_id UUID REFERENCES ops_reservation(id) ON DELETE SET NULL,
  family VARCHAR(64) NOT NULL,
  direction VARCHAR(32) NOT NULL,
  status VARCHAR(64) NOT NULL,
  subject VARCHAR(256),
  raw_text TEXT,
  parsed_summary_json TEXT,
  version BIGINT NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_notam (
  id UUID PRIMARY KEY,
  reservation_id UUID REFERENCES ops_reservation(id) ON DELETE SET NULL,
  notam_type VARCHAR(64),
  q_code VARCHAR(32),
  affected_location VARCHAR(32),
  lower_altitude_feet DOUBLE PRECISION,
  upper_altitude_feet DOUBLE PRECISION,
  effective_start TIMESTAMP WITH TIME ZONE,
  effective_end TIMESTAMP WITH TIME ZONE,
  raw_text TEXT,
  parsed_json TEXT,
  version BIGINT NOT NULL DEFAULT 0,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_apreq (
  id UUID PRIMARY KEY,
  reservation_id UUID REFERENCES ops_reservation(id) ON DELETE CASCADE,
  status VARCHAR(64) NOT NULL,
  request_text TEXT,
  response_text TEXT,
  version BIGINT NOT NULL DEFAULT 0,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_approval (
  id UUID PRIMARY KEY,
  reservation_id UUID REFERENCES ops_reservation(id) ON DELETE CASCADE,
  approval_type VARCHAR(64) NOT NULL,
  status VARCHAR(64) NOT NULL,
  actor VARCHAR(128),
  note TEXT,
  version BIGINT NOT NULL DEFAULT 0,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_weather_product (
  id VARCHAR(192) PRIMARY KEY,
  product_type VARCHAR(64) NOT NULL,
  provider VARCHAR(128),
  source_product VARCHAR(128),
  valid_start TIMESTAMP WITH TIME ZONE,
  valid_end TIMESTAMP WITH TIME ZONE,
  confidence DOUBLE PRECISION,
  raw_text TEXT,
  product_json TEXT NOT NULL,
  received_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_pirep (
  id VARCHAR(192) PRIMARY KEY,
  aircraft_id VARCHAR(64),
  aircraft_type VARCHAR(64),
  phenomenon VARCHAR(64),
  intensity VARCHAR(64),
  observation_time TIMESTAMP WITH TIME ZONE,
  latitude DOUBLE PRECISION,
  longitude DOUBLE PRECISION,
  altitude_feet DOUBLE PRECISION,
  raw_text TEXT,
  ingest_json TEXT NOT NULL,
  received_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_feed_artifact (
  id UUID PRIMARY KEY,
  source_id VARCHAR(128) NOT NULL,
  feed_type VARCHAR(64) NOT NULL,
  payload_hash VARCHAR(128) NOT NULL,
  raw_payload TEXT,
  diagnostics_json TEXT,
  accepted BOOLEAN NOT NULL,
  received_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_decision (
  id UUID PRIMARY KEY,
  action VARCHAR(64) NOT NULL,
  recommended_action VARCHAR(128),
  confidence DOUBLE PRECISION NOT NULL,
  rationale TEXT,
  result_json TEXT NOT NULL,
  audit_json TEXT,
  replay_json TEXT,
  version BIGINT NOT NULL DEFAULT 0,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE TABLE IF NOT EXISTS ops_reference_point (
  id UUID PRIMARY KEY,
  identifier VARCHAR(64) NOT NULL UNIQUE,
  point_type VARCHAR(64) NOT NULL,
  latitude DOUBLE PRECISION NOT NULL,
  longitude DOUBLE PRECISION NOT NULL,
  altitude_feet DOUBLE PRECISION,
  source VARCHAR(128),
  metadata_json TEXT
);

CREATE TABLE IF NOT EXISTS ops_history_event (
  id UUID PRIMARY KEY,
  aggregate_type VARCHAR(64) NOT NULL,
  aggregate_id VARCHAR(128) NOT NULL,
  event_type VARCHAR(128) NOT NULL,
  actor VARCHAR(128),
  note TEXT,
  event_json TEXT,
  created_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_ops_reservation_time ON ops_reservation(effective_start, effective_end);
CREATE INDEX IF NOT EXISTS idx_ops_weather_validity ON ops_weather_product(valid_start, valid_end);
CREATE INDEX IF NOT EXISTS idx_ops_feed_source_time ON ops_feed_artifact(source_id, received_at);
CREATE INDEX IF NOT EXISTS idx_ops_history_aggregate ON ops_history_event(aggregate_type, aggregate_id, created_at);
